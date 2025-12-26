import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

// This component wraps the entire app
// Perfect for adding global components like the chatbot
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
