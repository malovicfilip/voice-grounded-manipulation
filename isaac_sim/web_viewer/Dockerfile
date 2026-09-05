# Adapted from NVIDIA IsaacSim tools/docker/web-viewer/Dockerfile at
# 987015050efebfd0cd5d3736ae47fffe5adee308 (Apache-2.0).
# Browser only: this image does not start another simulator or robot controller.
FROM node:22-bookworm-slim AS builder
WORKDIR /app
RUN npm config set @nvidia:registry https://edge.urm.nvidia.com/artifactory/api/npm/omniverse-client-npm/ \
    && npx --yes @nvidia/create-ov-web-rtc-app@1.14.2 --name web-viewer --sample local-sample --outputDir /app
WORKDIR /app/web-viewer
ARG ISAACSIM_HOST
RUN test -n "$ISAACSIM_HOST" \
    && sed -i "s/signalingServer: [^,]*/signalingServer: '${ISAACSIM_HOST}'/" src/main.ts \
    && sed -i "/signalingServer:/a\\    signalingPort: 49100,\n    mediaServer: '${ISAACSIM_HOST}',\n    mediaPort: 47998,\n    forceWSS: false," src/main.ts \
    && npm install --ignore-scripts \
    && npm run build
FROM node:22-bookworm-slim
WORKDIR /app
COPY --from=builder /app/web-viewer/dist /app/dist
COPY server.mjs /app/server.mjs
USER node
EXPOSE 8210
CMD ["node", "/app/server.mjs"]
