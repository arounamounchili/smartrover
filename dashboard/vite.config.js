import { defineConfig } from 'vite';
import { viteSingleFile } from 'vite-plugin-singlefile';

const roverHost = process.env.ROVER_HOST || '192.168.4.1';

export default defineConfig({
  plugins: [viteSingleFile()],
  server: {
    proxy: {
      '/control': `http://${roverHost}`,
      '/status': `http://${roverHost}`,
      '/stream': `http://${roverHost}`,
    },
  },
  build: {
    reportCompressedSize: false,
  },
});
