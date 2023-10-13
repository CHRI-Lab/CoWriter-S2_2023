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

const ChildProfile = () => {
    const [firstName, setFirstName] = useState<string>('');
    const [lastName, setLastName] = useState<string>('');
    const [birthDate, setBirthDate] = useState<string>('');
    const [gender, setGender] = useState<string>('');
    const [handedness, setHandedness] = useState<string>('');

    const sendChildProfile = async () => {
        console.log(firstName, lastName, birthDate, gender, handedness);
        try {
            await axios.post('http://127.0.0.1:3001/manager/child_profile',
                {
                    "firstName": firstName,
                    "lastName": lastName,
                    "birthDate": birthDate,
                    "gender": gender,
                    "handedness": handedness,
                });
        } catch (error) {
            console.error('Error sending child profile to the backend:', error);
        }
    };

    return (
        <div>
            <div className="row">
                <div className="col">
                    <label>First Name: </label>
                    <input
                        type="text"
                        value={firstName}
                        onChange={(event) => setFirstName(event.target.value)}
                        placeholder="Enter First Name"
                    />
                </div>
            </div>
            <div className="row">
                <div className="col">
                    <label>Last Name: </label>
                    <input
                        type="text"
                        value={lastName}
                        onChange={(event) => setLastName(event.target.value)}
                        placeholder="Enter Last Name"
                    />
                </div>
            </div>
            <div className="row">
                <div className="col">
                    <label>Birth Date: </label>
                    <input
                        type="date"
                        value={birthDate}
                        onChange={(event) => setBirthDate(event.target.value)}
                    />
                </div>
            </div>
            <div className="row">
                <div className="col">
                    <label>Gender: </label>
                    <input
                        type="radio"
                        name="gender"
                        value="male"
                        onClick={() => setGender('male')}
                        checked={gender === "male"}
                    />
                    <label>Male</label>
                    <input
                        type="radio"
                        name="gender"
                        value="female"
                        onClick={() => setGender('female')}
                        checked={gender === "female"}
                    />
                    <label>Female</label>
                </div>
            </div>
            <div className="row">
                <div className="col">
                    <label>Handedness: </label>
                    <input
                        type="radio"
                        name="handedness"
                        value="right"
                        onClick={() => setHandedness('right')}
                        checked={handedness === "right"}
                    />
                    <label>Right Handed</label>
                    <input
                        type="radio"
                        name="handedness"
                        value="left"
                        onClick={() => setHandedness('left')}
                        checked={handedness === "left"}
                    />
                    <label>Left Handed</label>
                </div>
            </div>
            <div className="row">
                <div className="col">
                    <button onClick={sendChildProfile}>Submit</button>
                </div>
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
    const generateWord = async () => {
        try {
            await axios.post('http://127.0.0.1:3001/manager/generate_word');
        } catch (error) {
            console.error('Error sending generate_word to the backend:', error);
        }


        return (
            <div id="canvas_manager" className="container-fluid">
                <h1>Manager</h1>
                <WordToWriteInput />
                <GPTTextInput />
                <LearningPaceSlider />
                <button onClick={robotFinished}>Robot finished</button>
                <button onClick={stopRobot}>Stop</button>
                <button onClick={generateWord}>Generate Word</button>
                <button onClick={talkToMe}>Talk to me</button>
                <button onClick={erase}>Erase</button>

                <h2>Child Profile</h2>
                <ChildProfile />
            </div>
        );
    };

    export default CanvasManager;
