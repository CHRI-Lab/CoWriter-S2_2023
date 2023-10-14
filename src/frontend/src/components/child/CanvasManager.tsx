// import { useEffect, useState } from 'react';
// import axios from "axios";
// import '../css/canvas_manager.css'
// import Canvas from "./Canvas";

// import { UserInput } from "../../types/types";


// const CanvasManager = () => {
//     const [inputText, setInputText] = useState("");
//     const [generateCanvas, setGenerateCanvas] = useState(false);
//     const [userInputs, setUserInputs] = useState<UserInput[]>([]);

//     const updateUserInputs = (index: number, userInput: UserInput) => {
//         let updatedUserInputs = [...userInputs];
//         updatedUserInputs[index] = userInput;
//         setUserInputs(updatedUserInputs);
//     };

//     useEffect(() => {
//         setUserInputs([]);
//     }, [inputText]);

//     const sendStrokesToBackend = async () => {
//         console.log(userInputs)
//         try {
//             const response = await axios.post('http://127.0.0.1:5000/send_strokes', userInputs);

//             console.log(response.data);
//         } catch (error) {
//             console.error('Error sending strokes to the backend:', error);
//         }
//     };

//     const getWordDemo = async (word: string) => {
//         console.log(word)
//         try {
//             const response = await axios.post('http://127.0.0.1:5000/get_demo', { 'word': word });

//             console.log('reponse', response.data);
//         } catch (error) {
//             console.error('Error sending strokes to the backend:', error);
//         }
//     }

//     return (
//         <div id="canvas_manager" className="container-fluid">
//             <button type="button" className="btn btn-info" onClick={sendStrokesToBackend}>Done</button>
//             <button type="button" className="btn btn-warning" onClick={() => { setInputText("") }}>Clear</button>
//             <hr />
//             <input
//                 type="text"
//                 className="form-control"
//                 placeholder="input text"
//                 value={inputText}
//                 onChange={
//                     (event) => {
//                         setInputText(event.target.value);
//                         setGenerateCanvas(true);
//                         getWordDemo(event.target.value);
//                     }
//                 }
//             />
//             <hr />


//             <div className="row d-flex flex-row">
//                 {generateCanvas && Array.from(inputText).map((char, index) => (
//                     <div key={index} className="col-auto">
//                         <Canvas
//                             width={150}
//                             height={150}
//                             canvas_color={"#faf3e1"}
//                             char_to_draw={char}
//                             input_text={inputText}
//                             setUserInputs={(userInput: UserInput) => updateUserInputs(index, userInput)}
//                         />
//                     </div>
//                 ))}
//             </div>
//         </div>
//     )
// };


// export default CanvasManager;

import { useEffect, useState} from 'react';
import axios from "axios";
import '../css/canvas_manager.css'
import Canvas from "./Canvas";

import {UserInput} from "../../types/types";


const CanvasManager = () => {
    const [inputText, setInputText] = useState("");
    const [generateCanvas, setGenerateCanvas] = useState(false);
    const [userInputs, setUserInputs] = useState<UserInput[]>([]);
    const [imageURL, setImageURL] = useState("")

    const updateUserInputs = (index: number, userInput: UserInput) => {
        let updatedUserInputs = [...userInputs];
        updatedUserInputs[index] = userInput;
        setUserInputs(updatedUserInputs);
    };

    useEffect(() => {
        setUserInputs([]);
    }, [inputText]);

    // const submitText = async () => {
    //     setGenerateCanvas(true);

    //     console.log(inputText)
    //     try {
    //         const response = await axios.post('http://127.0.0.1:3001/child/send_text', {'inputText':inputText});

    //         console.log(response.data);
    //         setImageURL(response.data['image_url']);
    //     } catch (error) {
    //         console.error('Error submitting Test to the backend:', error);
    //     }
    // };


    const sendStrokesToBackend = async () => {
        console.log(userInputs)
        try {
            const response = await axios.post('http://127.0.0.1:3001/child/send_strokes', userInputs);

            console.log(response.data);
            setGenerateCanvas(false);
        } catch (error) {
            console.error('Error sending strokes to the backend:', error);
        }
    };

    const getWordDemo = async (word:string) => {
        console.log(word)
        try {
            const response = await axios.post('http://127.0.0.1:3001/child/get_demo', {'word':word});

            console.log('reponse', response.data);
        } catch (error) {
            console.error('Error sending strokes to the backend:', error);
        }
    };

    // const clearCanvas = async() => {
    //     setInputText("");
    //     setImageURL("");
    // };


    useEffect(() => {
        const updateImage = async () => {
            try {
                // Make axios request to the server to get the updated image URL
                const response = await axios.post('http://127.0.0.1:3001/child/update_url'); 

                console.log(response.data);
                setImageURL(response.data['image_url']);
                setInputText(response.data['text_to_write']);
                setGenerateCanvas(response.data['generate_canvas']);

            } catch (error) {
                console.error('Error fetching updated image:', error);
            }
        };

        // Update the image every 1000 milliseconds (5 seconds)
        const imageUpdateInterval = setInterval(updateImage, 5000);

        // Clear the interval when the component unmounts
        return () => clearInterval(imageUpdateInterval);
    }, []);

    return (

        <div id="canvas_manager" className="container-fluid">

            <img src= {imageURL} alt = "Image generated by OpenAI"/>
            <hr/>
            {/* <button type="button" className="btn btn-success" onClick={submitText}>Submit</button> */}
            {/* <button type="button" className="btn btn-warning" onClick={clearCanvas}>Clear</button> */}
            <button type="button" className="btn btn-info" onClick={sendStrokesToBackend}>Done</button>
            {/* <hr/>
            <input
                type="text"
                className="form-control"
                placeholder="input text"
                value={inputText}
                onChange={
                    (event) => {
                        setInputText(event.target.value);
                        // setGenerateCanvas(true);
                        getWordDemo(event.target.value);
                    }
                }
            />
            <hr/> */}

            <div className="row d-flex flex-row">
                {generateCanvas && Array.from(inputText).map((char, index) => (
                    <div key={index} className="col-auto">
                        <Canvas
                            width={150}
                            height={150}
                            canvas_color={"#faf3e1"}
                            char_to_draw={char}
                            input_text={inputText}
                            setUserInputs={(userInput: UserInput) => updateUserInputs(index, userInput)}
                        />
                    </div>
                ))}
            </div>
        </div>
    )
};


export default CanvasManager;