import './App.css'
import 'bootstrap/dist/css/bootstrap.min.css';
import CanvasManager from "./components/child/CanvasManager";
import ManagerUI from "./components/manager/ManagerUI";
import MainPage from './components/MainPage';

import { BrowserRouter as Router, Route, Routes } from 'react-router-dom'



function App() {
    return (
        <div className="wrapper">
            <Router>
                <Routes>
                    <Route path="/" element={<MainPage />} />
                    <Route path="/child" element={<CanvasManager />} />
                    <Route path="/manager" element={<ManagerUI />} />
                </Routes>
            </Router>
        </div>
    );
}

export default App
