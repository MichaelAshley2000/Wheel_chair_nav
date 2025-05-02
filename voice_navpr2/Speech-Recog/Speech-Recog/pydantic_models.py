from pydantic import BaseModel, Field

class Position(BaseModel):
    x: float = Field(
        ...,
        description="X coordinate of the position on the map."
    )
    y: float = Field(
        ...,
        description="Y coordinate of the position on the map."
    )

class ResponseFormat(BaseModel):
    transcript: str = Field(
        ...,
        description="The recognized or translated user input text."
    )
    location: str = Field(
        ...,
        description="The target location of the command. Location must be one of the following: 'kitchen', 'bed_room', 'toilet', 'living_room'."
    )
    command_type: str = Field(
        ...,
        description="The type of command, e.g., 'navigation'."
    )
    destination: Position = Field(
        ...,
        description="The target position (x, y) on the map. For kitchen its (4.0, 5.0). For bed_room its (3.0, 1.0)"
    )
    navigation_mode: str = Field(
        ...,
        description="Navigation mode, e.g., 'safe', 'fast', 'smooth'."
    )
    obstacle_avoidance: bool = Field(
        ...,
        description="Whether the system should perform obstacle avoidance. (True/False)"
    )
    priority: str = Field(
        ...,
        description="Priority level for this command, e.g., 'normal' or 'high'."
    )
