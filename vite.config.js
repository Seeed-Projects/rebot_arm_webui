import { defineConfig } from "vite";

export default defineConfig({
  server: {
    proxy: {
      "/sim": "http://localhost:8000",
      "/healthz": "http://localhost:8000",
      "/connect": "http://localhost:8000",
      "/disconnect": "http://localhost:8000",
      "/state": "http://localhost:8000",
      "/logs": "http://localhost:8000",
      "/move": "http://localhost:8000",
      "/home": "http://localhost:8000",
      "/gripper": "http://localhost:8000",
      "/permissions": "http://localhost:8000",
    },
  },
  build: {
    rollupOptions: {
      output: {
        manualChunks: {
          "three-core": ["three"],
          "three-extras": [
            "three/examples/jsm/controls/OrbitControls.js",
            "three/examples/jsm/loaders/STLLoader.js",
          ],
          urdf: ["urdf-loader"],
        },
      },
    },
  },
});
