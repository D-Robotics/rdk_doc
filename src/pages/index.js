// src/pages/index.js
import React, { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function HomeRedirect() {
  const history = useHistory();

  // useEffect(() => {
  //   // 重定向到外部 URL
  //   window.location.href = 'https://d-robotics.cc/';
  // }, []);
  const { i18n } = useDocusaurusContext();
  console.log(i18n.currentLocale)
  useEffect(() => {
    // 重定向到文档的首页路径
    if(i18n.currentLocale==="zh-Hans"){
      history.push('/rdk_doc/RDK');
    }else{
      history.push('/rdk_doc/en/RDK');
    }
      

  }, [history]);

  return null; // 不渲染任何内容
}

export default HomeRedirect;