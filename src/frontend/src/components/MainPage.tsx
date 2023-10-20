import { Link } from 'react-router-dom';

const MainPage = () => {
    return (
        <div id="canvas_manager" className="container-fluid">
            <h1>CoWriter Letter Learning</h1>
            <Link to="/child">
                <button type="button" className="btn btn-primary">Child</button>
            </Link>
            <Link to="/manager">
                <button type="button" className="btn btn-primary">Manager</button>
            </Link>
        </div>
    )
}

export default MainPage