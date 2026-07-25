import type { Metadata } from "next";
import { BoostLab } from "./BoostLab";

export const metadata: Metadata = {
  title: "Boost Control Lab",
  description: "STM32G431 Boost PI/PID 控制代码与交互调试练习台",
};

export default function Home() {
  return <BoostLab />;
}
