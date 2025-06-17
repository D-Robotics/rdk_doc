English| [简体中文](./README_CN.md)

Welcome to this project! This document will help you quickly get started with the installation, development, building, and deployment of RDK_DOC.

### I. Environment Installation

To install the dependencies for this project, execute the following command:

```shell
npm install
```

### II. Online Operation


To build only the Chinese manual:

```shell
npm run start
```

To build only the English manual:

```shell
npm run start  -- --locale en
```

This method does not support switching between Chinese and English documents. It can only build a single language document. If you need to display Chinese and English simultaneously, please refer to the method in Step III.

### III. Offline Deployment

To fully deploy the manual offline, please run the following script to download all images locally:

```shell
python3 download_imgs.py
```


For compiling and deploying the documents, use the following command:

```shell
npm run build
```

To deploy the documents, use the following command:

```shell
#Direct Deployment

npm run serve

#Deploy with Specified IP Address and Port Number

npm run serve -- --host=10.64.62.34 --port=1688 --no-open
```

This will start a static file server and provide the following links for access in the browser，The port number should be based on the actual port number:

***English manual link***: http://localhost:3000/en/rdk_doc/

***Chinese manual link***: http://localhost:3000/rdk_doc/

**Note:** Please ensure that Node.js version 18.0 or higher is required.

