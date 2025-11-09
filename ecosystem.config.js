module.exports = {
  apps: [
    {
      name: 'parol-commander',
      script: 'headless_commander.py',
      interpreter: 'python3',
      cwd: '/home/jacob/parol6/backend',
      env: {
        PYTHONUNBUFFERED: '1'
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
      cwd: '/home/jacob/parol6/backend',
      env: {
        PYTHONUNBUFFERED: '1'
      },
      autorestart: true,
      watch: false,
      max_memory_restart: '500M',
      error_file: '/home/jacob/.pm2/logs/parol-api-error.log',
      out_file: '/home/jacob/.pm2/logs/parol-api-out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss',
      merge_logs: true,
      instances: 1
    },
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
    }
  ]
};
