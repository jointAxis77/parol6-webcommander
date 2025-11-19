module.exports = {
  apps: [
    {
      name: 'parol-timeline',
      script: 'npm',
      args: 'run dev',
      watch: false, // Next.js has its own hot reload
      env: {
        NODE_ENV: 'development',
        PORT: 3000
      },
      error_file: './logs/err.log',
      out_file: './logs/out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss Z',
      autorestart: true,
      max_restarts: 10,
      min_uptime: '10s'
    }
  ]
};
