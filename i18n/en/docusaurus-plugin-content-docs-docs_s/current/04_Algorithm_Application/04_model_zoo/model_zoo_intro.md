---
sidebar_position: 1
---

# 4.4.1 ModelZoo Overview

:::tip

Model Zoo is still under development.

Github: https://github.com/D-Robotics/rdk_model_zoo_s/tree/s100
:::

Model Zoo aims to provide developers with a rich and diverse collection of ready-to-deploy model examples. Through this repository, developers can access the following resources:

1. **Diverse Digua heterogeneous models**: The repository includes various ready-to-deploy Digua heterogeneous models suitable for multiple scenarios and with strong generalization capabilities, covering domains such as image classification, object detection, semantic segmentation, natural language processing, and more. These .bin models have been carefully selected and optimized for high efficiency.
2. **Detailed usage guides**: Each model comes with a Jupyter Notebook containing comprehensive model descriptions, usage instructions, sample code, and annotations to help developers get started quickly. Additionally, for certain models, we also provide performance evaluation reports and tuning recommendations, enabling developers to customize and optimize models according to their specific needs.
3. **Integrated development tools**: We offer developers a set of Python APIs called `bpu_infer_lib` for rapid model deployment on RDK series development boards. By studying the Jupyter Notebooks accompanying the models in the repository—such as data preprocessing scripts and inference methods—developers can quickly master the use of this API, significantly simplifying the model development and deployment workflow.