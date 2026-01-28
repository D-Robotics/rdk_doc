window.difyChatbotConfig = {
  token: 'MltLQTHPb5EeP7uz',
  baseUrl: 'http://rdk.d-robotics.cc',
  inputs: {},
  systemVariables: {},
  userVariables: {},
};

/**
 * Dify Chatbot Draggable Logic
 * Enables the chatbot button to be dragged around the screen.
 * Keeps the chat window anchored relative to the button.
 */
(function () {
  const checkInterval = setInterval(() => {
    const button = document.getElementById('dify-chatbot-bubble-button');
    if (button) {
      clearInterval(checkInterval);
      makeDraggable(button);
    }
  }, 1000);

  function makeDraggable(element) {
    let isDragging = false;
    let startX, startY, startLeft, startTop;
    let hasMoved = false;

    // Ensure fixed positioning
    element.style.position = 'fixed';

    // Mouse events
    element.addEventListener('mousedown', (e) => {
      // Only left mouse button
      if (e.button !== 0) return;

      isDragging = true;
      hasMoved = false;
      startX = e.clientX;
      startY = e.clientY;

      const rect = element.getBoundingClientRect();
      startLeft = rect.left;
      startTop = rect.top;

      // Switch to left/top positioning for control
      element.style.right = 'auto';
      element.style.bottom = 'auto';
      element.style.left = startLeft + 'px';
      element.style.top = startTop + 'px';
      
      // Prevent text selection
      e.preventDefault();
    });

    // Touch support for mobile
    element.addEventListener('touchstart', (e) => {
      // Only single touch
      if (e.touches.length !== 1) return;

      isDragging = true;
      hasMoved = false;
      const touch = e.touches[0];
      startX = touch.clientX;
      startY = touch.clientY;

      const rect = element.getBoundingClientRect();
      startLeft = rect.left;
      startTop = rect.top;

      element.style.right = 'auto';
      element.style.bottom = 'auto';
      element.style.left = startLeft + 'px';
      element.style.top = startTop + 'px';
      
      // Prevent scrolling while starting drag
      e.preventDefault();
    }, { passive: false });

    window.addEventListener('touchmove', (e) => {
      if (!isDragging) return;
      
      const touch = e.touches[0];
      const dx = touch.clientX - startX;
      const dy = touch.clientY - startY;

      if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
        hasMoved = true;
      }

      element.style.left = (startLeft + dx) + 'px';
      element.style.top = (startTop + dy) + 'px';
      
      updateWindowPosition(element);
      
      // Prevent scrolling
      e.preventDefault();
    }, { passive: false });

    window.addEventListener('touchend', (e) => {
      if (!isDragging) return;
      isDragging = false;

      if (hasMoved) {
        // Prevent click if dragged
        const preventClick = (clickEvent) => {
          clickEvent.stopPropagation();
          clickEvent.stopImmediatePropagation();
          clickEvent.preventDefault();
          element.removeEventListener('click', preventClick, true);
        };
        element.addEventListener('click', preventClick, true);
      }
      updateWindowPosition(element);
    });

    window.addEventListener('mousemove', (e) => {
      if (!isDragging) return;

      const dx = e.clientX - startX;
      const dy = e.clientY - startY;

      // Threshold to detect drag vs click
      if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
        hasMoved = true;
        // Temporarily disable pointer events on iframe if any to prevent capturing mouse
        document.body.style.userSelect = 'none';
      }

      element.style.left = (startLeft + dx) + 'px';
      element.style.top = (startTop + dy) + 'px';

      // Update window position in real-time
      updateWindowPosition(element);
    });

    window.addEventListener('mouseup', (e) => {
      if (!isDragging) return;
      isDragging = false;
      document.body.style.userSelect = '';

      if (hasMoved) {
        // Intercept the click event to prevent toggling the chat window after a drag
        const preventClick = (clickEvent) => {
          clickEvent.stopPropagation();
          clickEvent.stopImmediatePropagation();
          clickEvent.preventDefault();
          element.removeEventListener('click', preventClick, true);
        };
        element.addEventListener('click', preventClick, true);
      }
      
      // Final position update
      updateWindowPosition(element);
    });
    
    // Also listen for clicks on the button to update window position (in case it was closed and re-opened)
    element.addEventListener('click', () => {
        // Small delay to allow the window to render
        setTimeout(() => updateWindowPosition(element), 50);
        setTimeout(() => updateWindowPosition(element), 200);
    });
  }

  function updateWindowPosition(button) {
    const chatWindow = document.getElementById('dify-chatbot-bubble-window');
    if (!chatWindow) return;

    const btnRect = button.getBoundingClientRect();
    const winRect = chatWindow.getBoundingClientRect();

    chatWindow.style.position = 'fixed';
    chatWindow.style.right = 'auto';
    chatWindow.style.bottom = 'auto';

    // Calculate position: Aligned to the Right edge of the button, Above the button
    let newLeft = btnRect.right - winRect.width;
    let newTop = btnRect.top - winRect.height - 16; // 16px gap

    // Boundary checks
    const viewportWidth = window.innerWidth;
    const viewportHeight = window.innerHeight;

    // 1. Horizontal check
    if (newLeft < 10) newLeft = 10; // Left boundary
    if (newLeft + winRect.width > viewportWidth) newLeft = viewportWidth - winRect.width - 10; // Right boundary

    // 2. Vertical check
    // If running off top, try placing below
    if (newTop < 10) {
        newTop = btnRect.bottom + 16;
        // If placing below runs off bottom, clamp to bottom
        if (newTop + winRect.height > viewportHeight) {
            newTop = viewportHeight - winRect.height - 10;
        }
    }

    chatWindow.style.left = newLeft + 'px';
    chatWindow.style.top = newTop + 'px';
  }
})();
