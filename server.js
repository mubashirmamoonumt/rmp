const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
const rateLimit = require('express-rate-limit');
const path = require('path');
const fs = require('fs');
const { v4: uuidv4 } = require('uuid');
const multer = require('multer');
const processor = require('./processor');
const archiver = require('archiver');
const cron = require('node-cron');

const app = express();
const PORT = process.env.PORT || 6000;

// Security Middlewares
app.use(helmet({
    crossOriginResourcePolicy: false,
}));
app.use(cors({
    origin: 'https://mute.m3toolskit.shop',
    methods: ['GET', 'POST'],
    allowedHeaders: ['Content-Type']
}));
app.use(express.json());

// Rate Limiting
const limiter = rateLimit({
    windowMs: 15 * 60 * 1000,
    max: 200
});
app.use(limiter);

const UPLOADS_DIR = path.join(__dirname, 'uploads');
if (!fs.existsSync(UPLOADS_DIR)) fs.mkdirSync(UPLOADS_DIR);

const storage = multer.diskStorage({
    destination: (req, file, cb) => {
        const sessionId = req.body.sessionId || uuidv4();
        req.sessionId = sessionId; // Pass to req for later use
        const sessionDir = path.join(UPLOADS_DIR, sessionId, 'input');
        if (!fs.existsSync(sessionDir)) fs.mkdirSync(sessionDir, { recursive: true });
        cb(null, sessionDir);
    },
    filename: (req, file, cb) => {
        cb(null, Date.now() + '-' + file.originalname.replace(/\s+/g, '_'));
    }
});

const upload = multer({
    storage: storage,
    limits: { fileSize: 100 * 1024 * 1024 },
    fileFilter: (req, file, cb) => {
        const filetypes = /mp4|avi|mov|mkv/;
        const extname = filetypes.test(path.extname(file.originalname).toLowerCase());
        const mimetype = filetypes.test(file.mimetype);
        if (mimetype && extname) return cb(null, true);
        cb(new Error('Only video files are allowed!'));
    }
});

// POST /upload
app.post('/api/upload', upload.array('videos', 50), (req, res) => {
    try {
        const sessionId = req.sessionId;
        req.files.forEach(file => {
            processor.addJob(sessionId, file.originalname, file.path);
        });

        res.status(200).json({
            message: 'Upload successful, processing started',
            sessionId
        });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// GET /status/:sessionId
app.get('/api/status/:sessionId', (req, res) => {
    const sessionId = req.params.sessionId;
    const jobs = processor.getJobStatus(sessionId);
    const overallProgress = processor.getOverallProgress(sessionId);

    res.json({
        sessionId,
        overallProgress,
        jobs
    });
});

// GET /download/:sessionId/:filename
app.get('/api/download/:sessionId/:filename', (req, res) => {
    const { sessionId, filename } = req.params;
    const filePath = path.join(UPLOADS_DIR, sessionId, 'output', `muted-${filename}`);

    if (fs.existsSync(filePath)) {
        res.download(filePath);
    } else {
        res.status(404).json({ error: 'File not found or still processing' });
    }
});

// GET /download-zip/:sessionId
app.get('/api/download-zip/:sessionId', (req, res) => {
    const sessionId = req.params.sessionId;
    const outputDir = path.join(UPLOADS_DIR, sessionId, 'output');

    if (!fs.existsSync(outputDir)) {
        return res.status(404).json({ error: 'No processed files found' });
    }

    const archive = archiver('zip', { zlib: { level: 9 } });
    res.attachment(`muted-videos-${sessionId}.zip`);

    archive.pipe(res);
    archive.directory(outputDir, false);
    archive.finalize();
});

// Cleanup Cron: Every 10 mins, delete folders older than 30 mins
cron.schedule('*/10 * * * *', () => {
    console.log('Running cleanup cron...');
    const now = Date.now();
    const expiration = 30 * 60 * 1000;

    fs.readdir(UPLOADS_DIR, (err, folders) => {
        if (err) return console.error('Cleanup error:', err);
        folders.forEach(folder => {
            const folderPath = path.join(UPLOADS_DIR, folder);
            fs.stat(folderPath, (err, stats) => {
                if (err) return;
                if (now - stats.mtimeMs > expiration) {
                    fs.rm(folderPath, { recursive: true, force: true }, (err) => {
                        if (err) console.error(`Failed to delete ${folder}:`, err);
                        else console.log(`Deleted expired session: ${folder}`);
                    });
                }
            });
        });
    });
});

app.listen(PORT, () => {
    console.log(`Server running on port ${PORT}`);
});
