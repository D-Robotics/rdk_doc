// src/pages/index.js
import React, { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';

function HomeRedirect() {
  const history = useHistory();

  // useEffect(() => {
  //   // 重定向到外部 URL
  //   window.location.href = 'https://d-robotics.cc/';
  // }, []);

  useEffect(() => {
    // 重定向到文档的首页路径
    history.push('/docs/');
  }, [history]);

  return null; // 不渲染任何内容
}

export default HomeRedirect;