import pygame
import pygame.freetype
import pygame.gfxdraw
import math

class GaugeWidget:
    def __init__(self, width, height, position=(200, 100)):
        pygame.init()
        self.width = width
        self.height = height
        self.position = position
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()

        self.widget_diameter = min(self.width, self.height)
        self.scale_angle_start_value = 135  # Start angle at 135 degrees
        self.scale_angle_size = 270  # Size of the gauge scale in degrees
        self.scalaCount = 10
        self.scala_subdiv_count = 1
        self.maxValue = 200
        self.minValue = 0
        self.value = 50
        self.speed = 0
        self.value_offset = 0
        self.angle_offset = 0
        self.scale_polygon_colors = [
            (0.5, (0, 0, 0)),  # Black for the inner part
            (1.0, (220, 220, 220)),  # Gray for the outer part
        ]
        self.gauge_color_outer_radius_factor = 0.85
        self.gauge_color_inner_radius_factor = 0.75
        self.bigScaleMarker = pygame.Color(0, 0, 0)
        self.fineScaleColor = pygame.Color(225, 215, 211)
        self.ScaleValueColor = pygame.Color(255, 255, 255)
        self.DisplayValueColor = pygame.Color(255, 255, 255)
        self.value_fontname = None
        self.value_fontsize = int(self.widget_diameter / 10)
        self.scale_fontname = None
        self.scale_fontsize = int(self.widget_diameter / 20)
        self.needle_center_bg = (255, 0, 0)
        self.outer_circle_bg = (255, 255, 255)
        self.NeedleColor = pygame.Color(255, 0, 0)
        self.value_needle = [[(self.width / 2, self.height / 2), (self.width / 2, self.height / 2 - 0.75 * self.widget_diameter)]]
        self.units = ""
        self.text_radius_factor = 0.5
        
    def render(self):
        # Create a new surface for the gauge with rounded corners

        arc_radius = 270  # Adjust for corner roundness

        # Draw other gauge elements directly on the screen
        self.draw_outer_circle()
        self.draw_big_scaled_marker()
        self.create_scale_marker_values_text()
        self.create_values_text()
        self.create_units_text()
        self.draw_needle()
        self.draw_big_needle_center_point()

        pygame.display.flip()  # Update the display

    def create_polygon_pie(self, outer_radius, inner_radius, start_angle, angle_size, fill=True, num_points=100):
        points = []
        num_steps = max(int(angle_size), 1)
        step = angle_size / num_steps
        for angle in range(int(start_angle), int(start_angle + angle_size + step), int(step)):
            x_outer = outer_radius * math.cos(math.radians(angle))
            y_outer = outer_radius * math.sin(math.radians(angle))
            points.append((self.width / 2 + x_outer, self.height / 2 + y_outer))
        if fill:
            points.append((self.width / 2, self.height / 2))
        return points
    
    def create_polygon_circle(self, radius, num_points=100):
        points = []
        for i in range(num_points):
            angle = i * (360 / num_points)
            x = self.width / 2 + radius * math.cos(math.radians(angle))
            y = self.height / 2 + radius * math.sin(math.radians(angle))
            points.append((x, y))
        return points


    def draw_filled_polygon(self, background_width, background_height, outline_pen_width=0):
        if self.scale_polygon_colors is not None:
            polygon_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)

            # Create a new surface for the background
            background_surface = pygame.Surface((background_width, background_height), pygame.SRCALPHA)

            # Define the arc radius for rounded corners
            arc_radius = 100  # Adjust this value as needed for the corner rounding

            # Draw the rectangle with rounded corners
            pygame.draw.rect(background_surface, (255, 0, 0), (0, 0, background_width, background_height), border_radius=arc_radius)

            # Blit the background_surface onto the polygon_surface
            polygon_surface.blit(background_surface, (0, 0))

            gradient_colors = [(119, 123, 127), (119, 123, 127), (119, 123, 127)]


            color_angles = [0, 120, 240]
            angle_sizes = [360, 360, 360]

            for i, (color, start_angle, angle_size) in enumerate(zip(gradient_colors, color_angles, angle_sizes)):
                outer_radius = ((self.widget_diameter / 2) - (outline_pen_width / 2)) * self.gauge_color_outer_radius_factor
                inner_radius = ((self.widget_diameter / 2) - (outline_pen_width / 2)) * self.gauge_color_inner_radius_factor
                colored_scale_polygon = self.create_polygon_pie(
                    outer_radius,
                    inner_radius,
                    start_angle,
                    angle_size
                )
                colored_scale_polygon_adjusted = [(x + self.position[0], y + self.position[1]) for x, y in colored_scale_polygon]
                pygame.draw.polygon(polygon_surface, color, colored_scale_polygon_adjusted, 0)

            self.screen.blit(polygon_surface, self.position)



    def draw_big_scaled_marker(self):
        line_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        color = (self.bigScaleMarker.r, self.bigScaleMarker.g, self.bigScaleMarker.b)
        steps_size = float(self.scale_angle_size) / float(self.scalaCount)
        scale_line_outer_start = self.widget_diameter / 2
        scale_line_length = (self.widget_diameter / 2) - (self.widget_diameter / 20)
        for i in range(self.scalaCount + 1):
            angle = self.scale_angle_start_value + i * steps_size
            end_pos = (scale_line_length * math.cos(math.radians(angle)), scale_line_length * math.sin(math.radians(angle)))
            pygame.draw.line(line_surface, color, (self.position[0] + self.width / 2, self.position[1] + self.height / 2), (self.position[0] + self.width / 2 + end_pos[0], self.position[1] + self.height / 2 + end_pos[1]), 2)

        self.screen.blit(line_surface, self.position)


    def create_scale_marker_values_text(self):
        if self.scale_fontname:
            font = pygame.freetype.Font(self.scale_fontname, self.scale_fontsize)
        else:
            font = pygame.freetype.SysFont(None, self.scale_fontsize)
        color = (self.ScaleValueColor.r, self.ScaleValueColor.g, self.ScaleValueColor.b)
        text_radius_factor = 0.65
        text_radius = self.widget_diameter / 2 * text_radius_factor
        scale_per_div = 20
        angle_distance = float(self.scale_angle_size) / float(self.scalaCount)
        for i in range(self.scalaCount + 1):
            value = self.minValue + scale_per_div * i
            text = str(int(value))
            rect = font.get_rect(text)
            w, h = rect.width, rect.height
            angle = math.radians(self.scale_angle_start_value + i * angle_distance)
            x = text_radius * math.cos(angle)
            y = text_radius * math.sin(angle)
            font.render_to(self.screen, (self.position[0] + self.width / 2 + x - w / 2, self.position[1] + self.height / 2 + y - h / 2), text, color)

    def create_values_text(self):
        font = pygame.freetype.Font(self.value_fontname, self.value_fontsize)
        color = (self.DisplayValueColor.r, self.DisplayValueColor.g, self.DisplayValueColor.b)
        text_radius = self.widget_diameter / 2 * self.text_radius_factor
        text = str(int(self.value))
        rect = font.get_rect(text)
        w, h = rect.width, rect.height
        font.render_to(self.screen, (self.position[0] + self.width / 2 - w / 2, self.position[1] + self.height / 2 - h / 2), text, color)

    def create_units_text(self):
        font = pygame.freetype.Font(self.value_fontname, int(self.value_fontsize / 2.5))
        color = (self.DisplayValueColor.r, self.DisplayValueColor.g, self.DisplayValueColor.b)
        text = str(self.units)
        rect = font.get_rect(text)
        w, h = rect.width, rect.height
        font.render_to(self.screen, (self.position[0] + self.width / 2 - w / 2, self.position[1] + self.height / 2 + h), text, color)


    def draw_outer_circle(self):
        surface = self.screen
        center_x = self.position[0] + self.width / 2
        center_y = self.position[1] + self.height / 2
        outer_radius = (self.widget_diameter / 2) * self.gauge_color_outer_radius_factor
        circle_thickness = 2
        pygame.draw.circle(surface, self.outer_circle_bg, (int(center_x), int(center_y)), int(outer_radius), int(circle_thickness))


    def draw_big_needle_center_point(self):
        surface = self.screen
        center_x = self.position[0] + self.width / 2
        center_y = self.position[1] + self.height / 2
        diameter = 45  # Adjust the diameter of the center point as desired
        thickness = 1  # Adjust the thickness of the outline as desired
        pygame.draw.circle(surface, (255, 255, 255), (int(center_x), int(center_y)), diameter + thickness // 2, thickness)


    def update_speed(self, speed):
        self.value = speed
    
    def set_position(self, position):
        self.position = position


    def draw_needle(self, scale_factor=2.5):
        surface = self.screen
        center_x = self.position[0] + self.width / 2
        center_y = self.position[1] + self.height / 2
        angle = ((self.value - self.value_offset - self.minValue) * self.scale_angle_size /
                (self.maxValue - self.minValue)) + self.scale_angle_start_value
        color = (self.NeedleColor.r, self.NeedleColor.g, self.NeedleColor.b)
       
        # Change the position of the needle for being closer to needle center
        # Calculate the end position of the needle based on a fixed length
        needle_length = min(self.width, self.height) * 0.4  # Adjust the needle length as desired
        start_pos = [
            center_x + ((needle_length - 140) * math.cos(math.radians(angle))),
            center_y + ((needle_length - 140) * math.sin(math.radians(angle)))
        ]
        end_pos = [
            center_x + ((needle_length - 15)* math.cos(math.radians(angle))),
            center_y + ((needle_length - 15)* math.sin(math.radians(angle)))
        ]
        pygame.draw.line(surface, color, start_pos, end_pos, 2)  # Reduce the thickness to 2


    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEMOTION:
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    angle = math.degrees(math.atan2(mouse_y - self.height / 2, mouse_x - self.width / 2))
                    normalized_angle = (angle - self.scale_angle_start_value) % 360
                    if normalized_angle < 0:
                        normalized_angle += 360
                    if normalized_angle <= self.scale_angle_size:
                        self.value = (normalized_angle / self.scale_angle_size) * (self.maxValue - self.minValue) + self.minValue
                        self.update_speed(self.value)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        self.value = min(self.maxValue, self.value + 20)
                        self.update_speed(self.value)
                    elif event.key == pygame.K_DOWN:
                        self.value = max(self.minValue, self.value - 20)
                        self.update_speed(self.value)

            self.screen.fill((0, 0, 0))
            # Draw a rectangle with rounded corners
            rect_surface = pygame.Surface((300, 200), pygame.SRCALPHA)
            pygame.draw.rect(rect_surface, (255, 0, 0), (0, 0, 300, 200), border_radius=50)
            self.screen.blit(rect_surface, (50, 50))
            self.render()
            pygame.display.flip()
            self.clock.tick(30)

        pygame.quit()

if __name__ == "__main__":
    widget = GaugeWidget(200, 100)
    widget.run()
