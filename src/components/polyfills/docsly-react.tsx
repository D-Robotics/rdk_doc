// Polyfill for @docsly/react
import React from 'react';

export const DyteButton = ({ children, kind = 'primary', size = 'md', ...props }) => {
  const baseClasses = 'px-4 py-2 rounded font-medium transition-colors';
  const kindClasses = {
    primary: 'bg-blue-600 text-white hover:bg-blue-700',
    secondary: 'bg-gray-200 text-gray-900 hover:bg-gray-300',
    ghost: 'bg-transparent text-blue-600 hover:bg-blue-50'
  };
  const sizeClasses = {
    sm: 'px-3 py-1 text-sm',
    md: 'px-4 py-2',
    lg: 'px-6 py-3 text-lg'
  };
  
  return (
    <button 
      className={`${baseClasses} ${kindClasses[kind]} ${sizeClasses[size]}`}
      {...props}
    >
      {children}
    </button>
  );
};

export const DyteIcon = ({ icon, size = 20, ...props }) => {
  return (
    <span 
      className="inline-block"
      style={{ width: size, height: size }}
      {...props}
    >
      {/* Placeholder for icon */}
      ⚡
    </span>
  );
};

export const DyteText = ({ children, size = 'md', weight = 'normal', ...props }) => {
  const sizeClasses = {
    sm: 'text-sm',
    md: 'text-base',
    lg: 'text-lg',
    xl: 'text-xl'
  };
  const weightClasses = {
    normal: 'font-normal',
    medium: 'font-medium',
    semibold: 'font-semibold',
    bold: 'font-bold'
  };
  
  return (
    <span 
      className={`${sizeClasses[size]} ${weightClasses[weight]}`}
      {...props}
    >
      {children}
    </span>
  );
};

export const DyteSpinner = ({ size = 20, ...props }) => {
  return (
    <div 
      className="animate-spin rounded-full border-2 border-gray-300 border-t-blue-600"
      style={{ width: size, height: size }}
      {...props}
    />
  );
};

export const DyteAvatar = ({ src, alt = '', size = 32, ...props }) => {
  return (
    <img 
      src={src} 
      alt={alt}
      className="rounded-full object-cover"
      style={{ width: size, height: size }}
      {...props}
    />
  );
};

export default {
  DyteButton,
  DyteIcon,
  DyteText,
  DyteSpinner,
  DyteAvatar
};
