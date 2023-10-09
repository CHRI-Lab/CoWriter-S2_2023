import React, { useState } from 'react';
import axios from "axios";


const CanvasManager = () => {
    const [wordInputValue, setWordInputValue] = useState<string>('');

    const handleValidationClick = async () => {
        console.log(wordInputValue)
        try {
            const response = await axios.post('http://127.0.0.1:3001/words_to_write', { "word": wordInputValue });
            console.log(response.data);
        } catch (error) {
            console.error('Error sending strokes to the backend:', error);
        }
    };

    return (
        <div id="canvas_manager" className="container-fluid">
            <h1>Manager</h1>

            <input
                type="text"
                value={wordInputValue}
                onChange={(event) => setWordInputValue(event.target.value)}
                placeholder="Enter text"
            />
            <button onClick={handleValidationClick}>Validate</button>
        </div>
    )
}

export default CanvasManager