# 使用 Node.js 官方镜像
FROM node:18-alpine

# 设置工作目录
WORKDIR /app

# 公开端口
EXPOSE 3000

# 默认命令: 构建和启动 Docusaurus 网站
CMD ["sh", "-c", "npm install && npm run build"]
