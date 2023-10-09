import { useState } from 'react';
import axios from 'axios';


const WordToWriteInput = () => {
    const [wordToWrite, setWordToWrite] = useState<string>('');

    const sendWordToWrite = async () => {
        console.log(wordToWrite);
        try {
            const response = await axios.post('http://127.0.0.1:3001/manager/words_to_write', { "word": wordToWrite });
            console.log(response.data);
        } catch (error) {
            console.error('Error sending word to the backend:', error);
        }
    };

    return (
        <div className="row">
            <div className="col">
                <input
                    type="text"
                    value={wordToWrite}
                    onChange={(event) => setWordToWrite(event.target.value)}
                    placeholder="Enter text"
                />
                <button onClick={sendWordToWrite}>Send</button>
            </div>
        </div>
    );
}

const GPTTextInput = () => {
    const [chatGPTText, setChatGPTText] = useState<string>('');

    const sendChatGPTText = async () => {
        console.log(chatGPTText);
        try {
            const response = await axios.post('http://127.0.0.1:3001/manager/gpt_text', { "text": chatGPTText });
            console.log(response.data);
        } catch (error) {
            console.error('Error sending text to the backend:', error);
        }
    };

    return (
        <div className="row">
            <div className="col">
                <input
                    type="text"
                    value={chatGPTText}
                    onChange={(event) => setChatGPTText(event.target.value)}
                    placeholder="Enter text"
                />
                <button onClick={sendChatGPTText}>Send</button>
            </div>
        </div>
    );
}

const LearningPaceSlider = () => {
    const [learningPace, setLearningPace] = useState<number>(50);

    const sendLearningPace = async () => {
        console.log(learningPace);
        try {
            await axios.post('http://127.0.0.1:3001/manager/learning_pace', { "pace": learningPace });
        } catch (error) {
            console.error('Error sending learning pace to the backend:', error);
        }
    };

    return (
        <div className="row">
            <div className="col">
                <input
                    type="range"
                    min="0"
                    max="100"
                    value={learningPace}
                    onChange={(event) => setLearningPace(parseInt(event.target.value))}
                    className="slider"
                    id="slider"
                />
                <button onClick={sendLearningPace}>Send</button>
            </div>
        </div>
    );
}


const CanvasManager = () => {
    const robotFinished = async () => {
        try {
            await axios.post('http://127.0.0.1:3001/manager/robot_finished');
        } catch (error) {
            console.error('Error sending robot finished to the backend:', error);
        }
    }
    const stopRobot = async () => {
        try {
            await axios.post('http://127.0.0.1:3001/manager/stop');
        } catch (error) {
            console.error('Error sending stop to the backend:', error);
        }
    }
    const talkToMe = async () => {
        try {
            await axios.post('http://127.0.0.1:3001/manager/talk_to_me');
        } catch (error) {
            console.error('Error sending talk_to_me to the backend:', error);
        }
    }
    const erase = async () => {
        try {
            await axios.post('http://127.0.0.1:3001/manager/erase');
        } catch (error) {
            console.error('Error sending erase to the backend:', error);
        }
    }

    return (
        <div id="canvas_manager" className="container-fluid">
            <h1>Manager</h1>
            <WordToWriteInput />
            <GPTTextInput />
            <LearningPaceSlider />
            <button onClick={robotFinished}>Robot finished</button>
            <button onClick={stopRobot}>Stop</button>
            <button onClick={talkToMe}>Talk to me</button>
            <button onClick={erase}>Erase</button>
        </div>
    );
};

export default CanvasManager;