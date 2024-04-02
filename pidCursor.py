import pygame
import sys
import pyautogui
import time

# Initialize Pygame
pygame.init()

# Set the width and height of the window
WINDOW_WIDTH, WINDOW_HEIGHT = 600, 400
WINDOW_SIZE = (WINDOW_WIDTH, WINDOW_HEIGHT)
# Initialize dot position to center of the screen
dot_x, dot_y = WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2

MAX_VELOCITY = 1000
# Set up the screen
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption("PID CTRLER")

# Get the screen resolution
SCREEN_WIDTH, SCREEN_HEIGHT = pyautogui.size()

# Function to scale mouse coordinates to fit within the window
def scale_coordinates(x, y):
    scaled_x = (x / SCREEN_WIDTH) * WINDOW_WIDTH
    scaled_y = (y / SCREEN_HEIGHT) * WINDOW_HEIGHT
    return scaled_x, scaled_y

# Define colors
BLACK = (0, 0, 0)
RED = (255, 0, 0)
WHITE = (255, 255, 255)

# Function to draw a dot at given coordinates
def draw_dot(x, y):
    pygame.draw.circle(screen, RED, (int(x), int(y)), 5)

# PID Controller Parameters
Kp = 0
Ki = 0
Kd = 0

# PID Controller Variables
prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0

# Function to calculate PID control output with maximum velocity
# Function to calculate PID control output with maximum velocity
def pid_control(target_x, target_y, current_x, current_y, dt):
    global prev_error_x, prev_error_y, integral_x, integral_y

    # Calculate errors
    error_x = target_x - current_x
    error_y = target_y - current_y

    # Update integral terms
    integral_x += error_x * dt
    integral_y += error_y * dt

    # Calculate derivative terms
    derivative_x = (error_x - prev_error_x) / dt
    derivative_y = (error_y - prev_error_y) / dt

    # Update previous errors
    prev_error_x = error_x
    prev_error_y = error_y

    # Calculate control outputs without considering maximum velocity
    output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
    output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

    # Limit control outputs to maximum velocity
    magnitude = (output_x ** 2 + output_y ** 2) ** 0.5
    if magnitude > MAX_VELOCITY:
        # Scale control outputs based on the ratio of new max velocity to previous max velocity
        scale_factor = MAX_VELOCITY / magnitude
        output_x *= scale_factor
        output_y *= scale_factor

    return output_x, output_y


# GUI Elements
slider_width = 10
slider_height = 80
slider_x = 20
slider_spacing = 70

# Slider rectangles
Kp_slider_rect = pygame.Rect(slider_x, 50, slider_width, slider_height)
Ki_slider_rect = pygame.Rect(slider_x + slider_spacing, 50, slider_width, slider_height)
Kd_slider_rect = pygame.Rect(slider_x + 2 * slider_spacing, 50, slider_width, slider_height)

# Text font
font = pygame.font.Font(None, 15)

# Main loop
def main():
    global Kp, Ki, Kd, dot_x, dot_y
    dot_x, dot_y = WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2

    try:
        while True:
            # Get the current position of the mouse cursor
            target_x, target_y = pyautogui.position()

            # Scale the mouse coordinates to fit within the window
            scaled_target_x, scaled_target_y = scale_coordinates(target_x, target_y)

            # Calculate time since last frame
            dt = 0.05  # Fixed time step of 0.05 seconds (20 FPS)

            # Calculate PID control outputs
            control_x, control_y = pid_control(scaled_target_x, scaled_target_y, dot_x, dot_y, dt)

            # Update dot position
            dot_x += control_x * dt
            dot_y += control_y * dt

            # Fill the screen with black color
            screen.fill(BLACK)

            # Draw a dot at the scaled dot position
            draw_dot(dot_x, dot_y)

            # Draw sliders
            pygame.draw.rect(screen, WHITE, Kp_slider_rect)
            pygame.draw.rect(screen, WHITE, Ki_slider_rect)
            pygame.draw.rect(screen, WHITE, Kd_slider_rect)

            # Draw slider handles
            pygame.draw.rect(screen, RED, (Kp_slider_rect.x, Kp_slider_rect.y + (1 - Kp) * slider_height, slider_width, 5))
            pygame.draw.rect(screen, RED, (Ki_slider_rect.x, Ki_slider_rect.y + (1 - Ki) * slider_height, slider_width, 5))
            pygame.draw.rect(screen, RED, (Kd_slider_rect.x, Kd_slider_rect.y + (1 - Kd) * slider_height, slider_width, 5))

            # Display PID values
            Kp_text = font.render(f"Kp: {Kp:.2f}", True, WHITE)
            Ki_text = font.render(f"Ki: {Ki:.2f}", True, WHITE)
            Kd_text = font.render(f"Kd: {Kd:.2f}", True, WHITE)
            screen.blit(Kp_text, (Kp_slider_rect.x + slider_width + 10, Kp_slider_rect.y))
            screen.blit(Ki_text, (Ki_slider_rect.x + slider_width + 10, Ki_slider_rect.y))
            screen.blit(Kd_text, (Kd_slider_rect.x + slider_width + 10, Kd_slider_rect.y))

            # Update the display
            pygame.display.flip()

            # Add a delay to achieve 20 FPS
            time.sleep(0.05)

            # Check for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # Check if the sliders are clicked
                    if Kp_slider_rect.collidepoint(event.pos):
                        Kp = 1 - (event.pos[1] - Kp_slider_rect.y) / slider_height
                    elif Ki_slider_rect.collidepoint(event.pos):
                        Ki = 1 - (event.pos[1] - Ki_slider_rect.y) / slider_height
                    elif Kd_slider_rect.collidepoint(event.pos):
                        Kd = 1 - (event.pos[1] - Kd_slider_rect.y) / slider_height

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
