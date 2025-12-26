import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [question, setQuestion] = useState('');
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Capture selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      if (text && text.length > 10 && text.length < 1000) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!question.trim()) return;

    // Add user message to chat
    const userMessage = {
      type: 'user',
      content: question,
      selectedContext: selectedText || null,
      timestamp: new Date().toISOString()
    };
    setMessages(prev => [...prev, userMessage]);

    // Clear input
    const currentQuestion = question;
    const currentSelection = selectedText;
    setQuestion('');
    setSelectedText('');
    setIsLoading(true);

    try {
      // Call backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_question: currentQuestion,
          selected_text: currentSelection || null
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to chat
      const botMessage = {
        type: 'bot',
        content: data.answer,
        sources: data.sources || [],
        model: data.model,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, botMessage]);

    } catch (error) {
      console.error('Chat error:', error);

      // Add error message
      const errorMessage = {
        type: 'error',
        content: `Error: ${error.message}. Make sure the backend server is running at http://localhost:8000`,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearSelectedText = () => {
    setSelectedText('');
  };

  const clearChat = () => {
    setMessages([]);
    setQuestion('');
    setSelectedText('');
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open AI Assistant"
        title="Ask the AI Assistant">
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Widget Panel */}
      {isOpen && (
        <div className={styles.chatWidget}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>AI Book Assistant</h3>
            <button onClick={clearChat} className={styles.clearButton} title="Clear chat">
              🗑️
            </button>
          </div>

          {/* Selected Text Indicator */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <div className={styles.selectedTextContent}>
                <strong>Selected:</strong> {selectedText.substring(0, 100)}
                {selectedText.length > 100 ? '...' : ''}
              </div>
              <button onClick={clearSelectedText} className={styles.clearSelectionButton}>
                ✕
              </button>
            </div>
          )}

          {/* Messages Area */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I'm your AI assistant for this book.</p>
                <p>Ask me anything about ROS 2, digital twins, Isaac, or VLA systems.</p>
                <p><em>Tip: Highlight text on the page and ask for more details!</em></p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${styles[msg.type]}`}>

                {msg.type === 'user' && (
                  <div className={styles.userMessage}>
                    {msg.selectedContext && (
                      <div className={styles.contextTag}>
                        📌 With selected text
                      </div>
                    )}
                    <p>{msg.content}</p>
                  </div>
                )}

                {msg.type === 'bot' && (
                  <div className={styles.botMessage}>
                    <div className={styles.answerText}>
                      {msg.content}
                    </div>
                    {msg.sources && msg.sources.length > 0 && (
                      <div className={styles.sources}>
                        <strong>Sources:</strong>
                        <ul>
                          {msg.sources.map((source, i) => (
                            <li key={i}>{source}</li>
                          ))}
                        </ul>
                      </div>
                    )}
                  </div>
                )}

                {msg.type === 'error' && (
                  <div className={styles.errorMessage}>
                    ⚠️ {msg.content}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={styles.loadingIndicator}>
                <div className={styles.typingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
                <p>Thinking...</p>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form onSubmit={handleSubmit} className={styles.inputArea}>
            <input
              type="text"
              value={question}
              onChange={(e) => setQuestion(e.target.value)}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask a question about the book..."}
              className={styles.input}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !question.trim()}>
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>
        </div>
      )}
    </>
  );
}
