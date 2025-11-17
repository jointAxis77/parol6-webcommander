module.exports = {
  apps: [
    {
      name: 'parol-nextjs',
      script: 'npm',
      args: 'run dev',
      cwd: '/home/jacob/parol6/frontend',
      env: {
        NODE_ENV: 'development',
        PORT: '3000'
      },
      autorestart: true,
      watch: false,  // Next.js has its own hot reload
      max_memory_restart: '1G',
      error_file: '/home/jacob/.pm2/logs/parol-nextjs-error.log',
      out_file: '/home/jacob/.pm2/logs/parol-nextjs-out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss',
      merge_logs: true,
      instances: 1
    },
    {
      name: 'parol-commander',
      script: 'commander.py',
      interpreter: 'python3',
      cwd: '/home/jacob/parol6/commander',
      env: {
        PYTHONUNBUFFERED: '1',
        PYTHONPATH: '/home/jacob/parol6'
      },
      autorestart: true,
      watch: false,
      max_memory_restart: '500M',
      error_file: '/home/jacob/.pm2/logs/parol-commander-error.log',
      out_file: '/home/jacob/.pm2/logs/parol-commander-out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss',
      merge_logs: true,
      instances: 1
    },
    {
      name: 'parol-api',
      script: 'fastapi_server.py',
      interpreter: 'python3',
      cwd: '/home/jacob/parol6/api',
      env: {
        PYTHONUNBUFFERED: '1',
        PYTHONPATH: '/home/jacob/parol6'
      },
      autorestart: true,
      watch: false,
      max_memory_restart: '500M',
      error_file: '/home/jacob/.pm2/logs/parol-api-error.log',
      out_file: '/home/jacob/.pm2/logs/parol-api-out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss',
      merge_logs: true,
      instances: 1
    }
  ]
};
