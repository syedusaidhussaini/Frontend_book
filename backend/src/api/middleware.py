"""
Custom middleware for FastAPI application
"""

import logging
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
import time
import json

logger = logging.getLogger(__name__)


class LoggingMiddleware:
    """Middleware to log all requests and responses"""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        request = Request(scope, receive)
        start_time = time.time()

        async def send_wrapper(message):
            if message["type"] == "http.response.start":
                status_code = message["status"]
                duration = time.time() - start_time
                logger.info(
                    f"{request.method} {request.url.path} - {status_code} - {duration:.3f}s"
                )
            await send(message)

        await self.app(scope, receive, send_wrapper)


def setup_middleware(app: FastAPI):
    """Setup all custom middleware for the application"""

    # Add logging middleware
    app.add_middleware(LoggingMiddleware)

    # Error handling
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        logger.error(f"Unhandled exception: {str(exc)}", exc_info=exc)
        return JSONResponse(
            status_code=500,
            content={
                "error": "Internal server error",
                "message": str(exc) if app.debug else "An error occurred",
            },
        )

    # 404 handler
    @app.exception_handler(404)
    async def not_found_handler(request: Request, exc):
        return JSONResponse(
            status_code=404,
            content={
                "error": "Not found",
                "path": request.url.path,
            },
        )

    logger.info("Middleware setup complete")
