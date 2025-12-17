"""
Pydantic models for chatbot query and response
"""

from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class Citation(BaseModel):
    """Citation pointing to a specific section in the book"""
    section_id: str = Field(..., description="Unique section identifier (e.g., '1.2.3')")
    chapter_title: str = Field(..., description="Chapter title")
    heading: str = Field(..., description="Section heading")
    line_range: Optional[str] = Field(None, description="Line range (start-end)")
    confidence_score: float = Field(..., ge=0.0, le=1.0, description="Confidence score 0-1")


class RetrievalMetadata(BaseModel):
    """Metadata about the retrieval process"""
    embedding_similarity: float = Field(..., description="Cosine similarity of embedding match")
    search_time_ms: int = Field(..., description="Time taken to search in milliseconds")
    num_contexts_retrieved: int = Field(..., description="Number of context sections retrieved")


class ChatbotQuery(BaseModel):
    """Request to the chatbot"""
    query: str = Field(..., description="User question or query", min_length=1)
    conversation_id: Optional[str] = Field(None, description="Conversation ID for context")
    selected_sections: Optional[List[str]] = Field(
        None,
        description="Optional: restrict answer to only these section IDs"
    )


class ChatbotResponse(BaseModel):
    """Response from the chatbot"""
    answer: str = Field(..., description="Chatbot's answer to the query")
    citations: List[Citation] = Field(..., description="List of source citations")
    conversation_id: str = Field(..., description="Conversation ID for tracking")
    retrieval_metadata: RetrievalMetadata = Field(..., description="Metadata about retrieval")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Response timestamp")


class ChatbotFeedback(BaseModel):
    """User feedback on chatbot response"""
    conversation_id: str = Field(..., description="Conversation ID")
    query_index: int = Field(..., description="Index of the query in conversation", ge=0)
    rating: int = Field(..., description="Rating 1-5", ge=1, le=5)
    comment: Optional[str] = Field(None, description="Optional user comment")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Feedback timestamp")
