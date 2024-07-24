// src/pages/index.js
import React, { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';

function HomeRedirect() {
  const history = useHistory();

  useEffect(() => {
    // 重定向到外部 URL
    window.location.href = 'https://developer.d-robotics.cc/';
  }, []);

  return null; // 不渲染任何内容
}

export default HomeRedirect;