const ffmpeg = require('fluent-ffmpeg');
const path = require('path');
const fs = require('fs');

class Processor {
    constructor(concurrency = 3) {
        this.queue = [];
        this.activeWorkers = 0;
        this.maxWorkers = concurrency;
        this.jobs = new Map(); // jobId -> status
    }

    addJob(sessionId, filename, inputPath) {
        const jobId = `${sessionId}-${filename}`;
        const outputPath = path.join(path.dirname(inputPath), '..', 'output');
        if (!fs.existsSync(outputPath)) fs.mkdirSync(outputPath, { recursive: true });

        const outputFilePath = path.join(outputPath, `muted-${filename}`);

        const job = {
            id: jobId,
            sessionId,
            filename,
            inputPath,
            outputPath: outputFilePath,
            status: 'queued',
            progress: 0
        };

        this.jobs.set(jobId, job);
        this.queue.push(job);
        this.processNext();
        return jobId;
    }

    processNext() {
        if (this.activeWorkers >= this.maxWorkers || this.queue.length === 0) return;

        const job = this.queue.shift();
        job.status = 'processing';
        this.activeWorkers++;

        ffmpeg(job.inputPath)
            .noAudio()
            .on('progress', (progress) => {
                job.progress = Math.round(progress.percent || 0);
            })
            .on('end', () => {
                job.status = 'completed';
                job.progress = 100;
                this.activeWorkers--;
                this.processNext();
            })
            .on('error', (err) => {
                console.error(`Error processing ${job.filename}:`, err.message);
                job.status = 'failed';
                job.error = err.message;
                this.activeWorkers--;
                this.processNext();
            })
            .save(job.outputPath);
    }

    getJobStatus(sessionId) {
        const sessionJobs = [];
        for (const job of this.jobs.values()) {
            if (job.sessionId === sessionId) {
                sessionJobs.push({
                    name: job.filename,
                    status: job.status,
                    progress: job.progress,
                    error: job.error
                });
            }
        }
        return sessionJobs;
    }

    getOverallProgress(sessionId) {
        const jobs = this.getJobStatus(sessionId);
        if (jobs.length === 0) return 0;
        const totalProgress = jobs.reduce((sum, job) => sum + job.progress, 0);
        return Math.round(totalProgress / jobs.length);
    }
}

module.exports = new Processor();
