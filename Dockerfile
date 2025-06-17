# Use the official Node.js image
FROM node:18-alpine

# Set the working directory
WORKDIR /app

# Expose the port
EXPOSE 3000

# Default command: Build and start the Docusaurus site
CMD ["sh", "-c", "npm run build"]

# Add environment setup option, refer to the usage below
# CMD ["sh", "-c", "npm install && npm run build && npm run serve"]
