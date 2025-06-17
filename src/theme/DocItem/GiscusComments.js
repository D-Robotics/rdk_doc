import React from 'react';
import Giscus from '@giscus/react';

function GiscusComments() {
  return (
        <Giscus
         repo="D-Robotics/rdk_doc"
         repoId="R_kgDOMaIl2w"

        //  repoId="806560652"
         category="General"
        //  categoryId="42103519"
         categoryId="DIC_kwDOMaIl284ChSxo"

         mapping="pathname"
         reactionsEnabled="1"
         emitMetadata="1"
         inputPosition="top"
         theme="light"
         lang="en"
         loading="auto"
          />
        );
      }


export default GiscusComments;