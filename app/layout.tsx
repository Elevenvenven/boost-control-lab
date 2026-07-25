import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Boost Control Lab",
  description: "STM32G431 Boost PI/PID 控制代码与交互调试练习台",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="zh-CN">
      <body>{children}</body>
    </html>
  );
}
