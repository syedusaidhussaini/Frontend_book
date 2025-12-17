"""
FastAPI application for ROS 2 Book RAG Chatbot Backend
Handles RAG orchestration, chatbot queries, and conversation management
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

from config import settings
from api.middleware import setup_middleware
from api.routers import chatbot, simulation, sensor_simulation, unity_integration, simulation_coordination

# Configure logging
logging.basicConfig(
    level=settings.log_level,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="ROS 2 Book RAG Chatbot API",
    description="API for RAG-based chatbot answering questions about ROS 2 book content",
    version="1.0.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Setup custom middleware
setup_middleware(app)

# Include routers
app.include_router(chatbot.router, prefix="/chatbot", tags=["chatbot"])
app.include_router(simulation.router, prefix="/v1", tags=["simulation"])
app.include_router(sensor_simulation.router, prefix="/v1", tags=["sensor_simulation"])
app.include_router(unity_integration.router, prefix="/v1", tags=["unity_integration"])
app.include_router(simulation_coordination.router, prefix="/v1", tags=["simulation_coordination"])

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "environment": settings.environment,
        "version": "1.0.0"
    }

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "ROS 2 Book RAG Chatbot API",
        "documentation": "/docs",
        "health": "/health"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug,
        log_level=settings.log_level.lower()
    )
