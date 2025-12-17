"""
Configuration module for FastAPI backend
Handles environment variables and settings using Pydantic
"""

from pydantic_settings import BaseSettings
from typing import List
import logging

logger = logging.getLogger(__name__)


class Settings(BaseSettings):
    """Application settings from environment variables"""

    # Environment
    environment: str = "development"
    debug: bool = True
    log_level: str = "INFO"

    # Server
    host: str = "0.0.0.0"
    port: int = 8000

    # OpenAI Configuration
    openai_api_key: str = ""
    openai_model: str = "gpt-4"
    openai_embedding_model: str = "text-embedding-3-small"

    # Qdrant Vector Database
    qdrant_url: str = ""
    qdrant_api_key: str = ""
    qdrant_collection_name: str = "ros2_chapters"

    # Neon Postgres Database
    database_url: str = ""

    # Chatbot Configuration
    chatbot_similarity_threshold: float = 0.7
    chatbot_max_context_tokens: int = 2000
    chatbot_temperature: float = 0.7

    # CORS Configuration
    allowed_origins: List[str] = [
        "http://localhost:3000",
        "http://localhost:8000",
        "http://localhost:5173",
    ]

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    def __init__(self, **data):
        super().__init__(**data)
        # Validate critical settings
        if self.environment == "production":
            if not self.openai_api_key:
                raise ValueError("OPENAI_API_KEY must be set in production")
            if not self.qdrant_api_key:
                raise ValueError("QDRANT_API_KEY must be set in production")
            if not self.database_url:
                raise ValueError("DATABASE_URL must be set in production")
        logger.info(f"Settings loaded for {self.environment} environment")


# Global settings instance
settings = Settings()
