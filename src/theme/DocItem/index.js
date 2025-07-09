import React from 'react';
import DocItem from '@theme-original/DocItem';
import GiscusComments from './GiscusComments';

export default function DocItemWrapper(props) {
  return (
    <>
      <DocItem {...props} />
      <GiscusComments />
    </>
  );
}
