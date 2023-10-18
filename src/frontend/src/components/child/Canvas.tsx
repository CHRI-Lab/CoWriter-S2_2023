import { useCallback, useEffect, useRef, useState } from 'react';
import '../css/canvas.css'
import { Coordinate, Stroke, UserInput } from "../../types/types";

interface CanvasProps {
    width: number;
    height: number;
    canvas_color: string;
    char_to_draw: string;
    input_text: string;
    setUserInputs: (userInput: UserInput) => void;
}

const Canvas = ({ width, height, canvas_color, char_to_draw, input_text, setUserInputs }: CanvasProps) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const [isPainting, setIsPainting] = useState(false);
    const [mousePosition, setMousePosition] = useState<Coordinate | undefined>(undefined);
    const [strokes, setStrokes] = useState<Stroke[]>([]);

    const startPaint = useCallback((event: MouseEvent) => {
        const coordinates = getCoordinates(event);
        if (coordinates) {
            setMousePosition(coordinates);
            setIsPainting(true);
            setStrokes((prevStrokes) => [...prevStrokes, []]);
        }
    }, []);

    useEffect(() => {
        if (!canvasRef.current) {
            return;
        }
        const canvas: HTMLCanvasElement = canvasRef.current;
        canvas.addEventListener('mousedown', startPaint);
        return () => {
            canvas.removeEventListener('mousedown', startPaint);
        };
    }, [startPaint]);

    const paint = useCallback(
        (event: MouseEvent) => {
            if (isPainting) {
                const newMousePosition = getCoordinates(event);
                if (mousePosition && newMousePosition) {
                    drawLine(mousePosition, newMousePosition);
                    setMousePosition(newMousePosition);
                    setStrokes((prevStrokes) => {
                        const currentStroke = prevStrokes[prevStrokes.length - 1];
                        const updatedStroke = [...currentStroke, newMousePosition];
                        const updatedStrokes = prevStrokes.slice(0, prevStrokes.length - 1);
                        return [...updatedStrokes, updatedStroke];
                    });
                    setUserInputs(get_user_input())
                }
            }
        },
        [isPainting, mousePosition]
    );

    useEffect(() => {
        if (!canvasRef.current) {
            return;
        }
        const canvas: HTMLCanvasElement = canvasRef.current;
        canvas.addEventListener('mousemove', paint);
        return () => {
            canvas.removeEventListener('mousemove', paint);
        };
    }, [paint]);

    const exitPaint = useCallback(() => {
        setIsPainting(false);
        setMousePosition(undefined);
    }, []);

    useEffect(() => {
        if (!canvasRef.current) {
            return;
        }
        const canvas: HTMLCanvasElement = canvasRef.current;
        canvas.addEventListener('mouseup', exitPaint);
        canvas.addEventListener('mouseleave', exitPaint);
        return () => {
            canvas.removeEventListener('mouseup', exitPaint);
            canvas.removeEventListener('mouseleave', exitPaint);
        };
    }, [exitPaint]);

    const getCoordinates = (event: MouseEvent): Coordinate | undefined => {
        if (!canvasRef.current) {
            return;
        }

        const canvas: HTMLCanvasElement = canvasRef.current;
        return { x: event.pageX - canvas.offsetLeft, y: event.pageY - canvas.offsetTop };
    };

    const drawLine = (originalMousePosition: Coordinate, newMousePosition: Coordinate) => {
        if (!canvasRef.current) {
            return;
        }
        const canvas: HTMLCanvasElement = canvasRef.current;
        const context = canvas.getContext('2d');
        if (context) {
            context.strokeStyle = 'red';
            context.lineJoin = 'round';
            context.lineWidth = 5;

            context.beginPath();
            context.moveTo(originalMousePosition.x, originalMousePosition.y);
            context.lineTo(newMousePosition.x, newMousePosition.y);
            context.closePath();

            context.stroke();
        }
    };

    const get_user_input = () => {
        let user_input: UserInput = {
            strokes: strokes,
            shape_id: null,
            shape_type: char_to_draw,
            shapetype_code: null,
            params_to_vary: null,
            param_values: null,
        };
        // console.log(user_input);
        return user_input;
    };

    const clearCanvas = () => {
        if (!canvasRef.current) {
            return;
        }
        const canvas: HTMLCanvasElement = canvasRef.current;
        const context = canvas.getContext('2d');
        if (context) {
            context.clearRect(0, 0, canvas.width, canvas.height);
        }
        setStrokes([])
    };

    useEffect(() => {
        clearCanvas();
    }, [input_text]);

    // const handelDown = () =>{
    //     const imaageUrl = canvasRef.current.toDataURL("image/png");
    //     const blob = base64ToBlob(imaageUrl);
    //     const link = document.createElement("a");
    //     link.href = URL.createObjectURL(blob);
    //     link.download = "a.png";
    //     const clickHandler = new MouseEvent("click");
    //     link.dispatchEvent(clickHandler)
    // }


    return (
        <div id="canvas_container">
            <button type="button" className="btn btn-warning" onClick={clearCanvas}>Rewrite</button>
            {/* <button onClick = {handelDown}>Feedback</button> */}
            <div>{char_to_draw}</div>
            <canvas ref={canvasRef} height={height} width={width} style={{ backgroundColor: canvas_color }} />

        </div>
    )
};

// Canvas.defaultProps = {
//     width: window.innerWidth,
//     height: window.innerHeight-100,
//     canvas_color: "#faf3e1"
// };




export default Canvas;
