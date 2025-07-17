import React, { useEffect } from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';

import ErrorBoundary from '@docusaurus/ErrorBoundary';
import {
  PageMetadata,
  SkipToContentFallbackId,
  ThemeClassNames,
} from '@docusaurus/theme-common';
import { useKeyboardNavigation } from '@docusaurus/theme-common/internal';
import SkipToContent from '@theme/SkipToContent';
import AnnouncementBar from '@theme/AnnouncementBar';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorPageContent from '@theme/ErrorPageContent';
import styles from './styles.module.css';

export default function Layout(props) {
  const {
    children,
    noFooter,
    wrapperClassName,
    title,
    description,
  } = props;

  const location = useLocation();
  useKeyboardNavigation();

  useEffect(() => {
    // ✅ 只在浏览器端执行 medium-zoom 初始化
    if (typeof window !== 'undefined') {
      import('medium-zoom').then((mediumZoom) => {
        const zoom = mediumZoom.default({ background: '#000', margin: 24 });
        zoom.attach('article img');
      });
    }
  }, [location.pathname]); // 路由变化时重新 attach

  return (
    <LayoutProvider>
      <PageMetadata title={title} description={description} />

      <SkipToContent />
      <AnnouncementBar />
      <Navbar />

      <div
        id={SkipToContentFallbackId}
        className={clsx(
          ThemeClassNames.layout.main.container,
          ThemeClassNames.wrapper.main,
          styles.mainWrapper,
          wrapperClassName,
        )}>
        <ErrorBoundary fallback={(params) => <ErrorPageContent {...params} />}>
          {children}
        </ErrorBoundary>
      </div>

      {!noFooter && <Footer />}
    </LayoutProvider>
  );
}
