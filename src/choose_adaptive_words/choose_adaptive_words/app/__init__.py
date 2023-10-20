from flask import Flask
from flask_cors import CORS

from .manager import MANAGER_BP
from .child import CHILD_BP


def create_app():
    app = Flask(__name__)

    cors_config = {
        "origins": ["*"],
        "methods": ["GET", "POST", "PUT", "DELETE"],
        "allow_headers": ["Content-Type", "Authorization"],
    }
    CORS(app, resources={r"*": cors_config})

    app.register_blueprint(MANAGER_BP)
    app.register_blueprint(CHILD_BP)
    return app
