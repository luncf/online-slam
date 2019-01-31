import math
import sys
from copy import deepcopy

import numpy as np
import rospy

class Particle(object):

    def __init__(self, map_rows, map_columns, row, column, angle, probability):
        self.map_rows = map_rows
        self.map_columns = map_columns

        self.row = row
        self.column = column
        self.angle = angle
        self.probability = probability
        self.valid = True

        self.randomize(row=row, column=column, angle=angle, probability=probability)

    def randomize(self, row, column, angle, probability):
        # Generate a particle nearby the belief row and column that is within the map
        new_row, new_column = self.generate_normal(row=row, column=column, angle=angle, probability=probability)
        while not self.in_map(row=new_row, column=new_column):
            new_row, new_column = self.generate_normal(row=row, column=column, angle=angle, probability=probability)
        
        # Assign values
        self.row = new_row
        self.column = new_column
        self.angle = angle
        self.probability = probability
        self.valid = True

    def generate_normal(self, row, column, angle, probability):
        row_scale = 0.6 - probability
        column_scale = 0.6 - probability

        if angle == 0 or angle == 2:
            row_scale /= 2.0
        elif angle == 1 or angle ==3:
            column_scale /= 2.0

        return int(round(np.random.normal(row, scale=row_scale if row_scale > 0 else 0.01))), \
               int(round(np.random.normal(column, scale=column_scale if column_scale > 0 else 0.01)))

    def move(self, estimated_num_cells):
        row = self.row
        column = self.column

        # Move particle for estimated steps in angle direction
        if self.angle == 0:
            column += estimated_num_cells
        elif self.angle == 1:
            row -= estimated_num_cells
        elif self.angle == 2:
            column -= estimated_num_cells
        elif self.angle == 3:
            row += estimated_num_cells

        # Check whether particle is still within map
        if self.in_map(row=row, column=column):
            self.row = row
            self.column = column
        else:
            self.probability = 0.0
            self.valid = False

    def in_map(self, row, column):
        return 0 <= row < self.map_rows and 0 <= column < self.map_columns


class Particles(object):

    def __init__(self, row, column, angle, map_rows, map_columns, num_particles=5):
        self.particles = [Particle(map_rows=map_rows, map_columns=map_columns, row=row, column=column,
                                   angle=angle, probability=1.0 / num_particles)
                          for _ in range(num_particles)]

    def even_probability(self):
        return 1.0 / len(self.particles)

    def randomize(self, row, column, angle):
        probability = self.even_probability()
        for particle in self.particles:
            particle.randomize(row=row, column=column, angle=angle, probability=probability)
    
    def random_sample(self, row, column, estimated_num_cells, probability_map):
        best_candidate = None

        # Best candidate will be on highest probability of probability map
        if probability_map is not None:
            best_region = None
            shortest_distance = sys.maxsize
            for region_row, region_column in probability_map.regions:
                distance = math.sqrt(math.fabs(region_row - row)**2 + math.fabs(region_column - column)**2)
                if distance < shortest_distance:
                    best_region = (region_row, region_column)
                    shortest_distance = distance
            if best_region is not None:
                best_candidate = self.particles[0]
                best_candidate.row, best_candidate.column = best_region
                best_candidate.probability = probability_map.map[best_candidate.row][best_candidate.column]
                best_candidate.valid = True

        # Pick the best candidate based on distance to the previous best candidate
        if best_candidate is None:
            shortest_distance = sys.maxsize
            best_particle = None

            for particle in self.particles:
                distance = math.sqrt(math.fabs(particle.row - row)**2 + math.fabs(particle.column - column)**2)
                if distance < shortest_distance:
                    shortest_distance = distance
                    best_particle = particle
                elif (distance == shortest_distance) and \
                        ((particle.angle == 0 and particle.column - column > best_particle.column - column) or \
                            (particle.angle == 1 and particle.row - row < best_particle.row - row) or \
                            (particle.angle == 2 and particle.column - column < best_particle.column - column) or \
                            (particle.angle == 3 and particle.row - row > best_particle.row - row)):
                    best_particle = particle

            best_candidate = best_particle

        # Trim number of particles based on best candidate probability
        if best_candidate.probability > self.even_probability() and len(self.particles) > 1:
            self.particles.pop(0)


        # if best_candidate.probability > self.even_probability():
        #     # Trim number of particles based on best candidate probability
        #     if len(self.particles) > 1:
        #         self.particles.pop(0)
        # else:
        #     # Pick the best candidate based on distance to the previous best candidate
        #     shortest_distance = sys.maxsize
        #     best_particle = None

        #     for particle in self.particles:
        #         distance = math.sqrt(math.fabs(particle.row - row)**2 + math.fabs(particle.column - column)**2)
        #         if distance < shortest_distance:
        #             shortest_distance = distance
        #             best_particle = particle
        #         elif (distance == shortest_distance) and \
        #                 ((particle.angle == 0 and particle.column - column > best_particle.column - column) or \
        #                     (particle.angle == 1 and particle.row - row < best_particle.row - row) or \
        #                     (particle.angle == 2 and particle.column - column < best_particle.column - column) or \
        #                     (particle.angle == 3 and particle.row - row > best_particle.row - row)):
        #             best_particle = particle

        #     best_candidate = best_particle

        for particle in self.particles:
            # Randomize based on best candidate origin
            # Scale of randomization is based on probability of best candidate
            particle.randomize(row=best_candidate.row, column=best_candidate.column,
                               angle=best_candidate.angle, probability=best_candidate.probability)

            # Move each particle for the estimated number of cells in angle direction
            particle.move(estimated_num_cells=estimated_num_cells)
        
            # Update probability based on probability map
            if particle.valid and probability_map is not None:
                particle.probability = probability_map.map[int(particle.row)][int(particle.column)]
            else:
                particle.probability = 0.0

        # Normalize probabilities of particles
        total_probability = sum(particle.probability for particle in self.particles)
        if total_probability > 0.0:
            for particle in self.particles:
                particle.probability = particle.probability / float(total_probability)
        else:
            for particle in self.particles:
                particle.probability = self.even_probability()

        return best_candidate
