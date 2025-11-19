'use client';

import "./globals.css";
import WebSocketConnector from './components/WebSocketConnector';

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="dark">
      <head>
        <title>PAROL6 Control</title>
        <link href="https://fonts.googleapis.com/icon?family=Material+Icons&display=block" rel="stylesheet" />
      </head>
      <body className="antialiased">
        {/* Global WebSocket connection - updates Zustand store on all pages */}
        <WebSocketConnector />
        {children}
      </body>
    </html>
  );
}
