import bpy

cursor_location = bpy.context.scene.cursor.location
log_text = f"{cursor_location.x}, {cursor_location.y}, {cursor_location.z}\n"

# Create or get a text block named 'CursorLog'
text_block = bpy.data.texts.get("CursorLog") or bpy.data.texts.new("CursorLog")
text_block.write(log_text)
