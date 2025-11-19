/** @type {import('next').NextConfig} */
const nextConfig = {
  // Disable React Strict Mode for robotics interface to prevent duplicate WebSocket connections
  reactStrictMode: false,

  webpack: (config, { isServer }) => {
    // Fix for urdf-loader and three.js in webpack
    config.module.rules.push({
      test: /\.urdf$/,
      type: 'asset/source'
    });

    // Enable polling for file watching on WSL + Windows mounts (drvfs)
    // This is necessary because Windows filesystems don't support inotify
    if (!isServer) {
      config.watchOptions = {
        poll: 1000, // Check for changes every 1 second
        aggregateTimeout: 300, // Delay before rebuilding after first change
        ignored: /node_modules/, // Don't watch node_modules
      };
    }

    return config;
  }
};

module.exports = nextConfig;
