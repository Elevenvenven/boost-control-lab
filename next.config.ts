import type { NextConfig } from "next";

const pagesBasePath = process.env.NEXT_PUBLIC_BASE_PATH ?? "";

const nextConfig: NextConfig = {
  output: "export",
  trailingSlash: true,
  basePath: pagesBasePath,
  assetPrefix: pagesBasePath || undefined,
};

export default nextConfig;
