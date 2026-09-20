# Implementacja logiki roju w oparciu o model Cuckera-Smale'a
import math

class CuckerSmaleBoid:
    """Pojedyncza jednostka w roju."""
    def __init__(self, id, initial_position, initial_velocity):
        self.id = id
        self.position = list(initial_position)
        self.velocity = list(initial_velocity)
        
    def update_velocity(self, boids, dt, alpha=1.0, beta=0.5):
        """
        Zaktualizuj prędkość na podstawie sąsiadów.
        Model Cuckera-Smale'a zakłada dopasowywanie prędkości (alignment) na podstawie
        funkcji odległości między agentami.
        """
        new_v = list(self.velocity)
        for boid in boids:
            if boid.id == self.id: continue
            
            # Odległość
            dist_sq = sum((self.position[i] - boid.position[i])**2 for i in range(3))
            
            # Waga wpływu
            weight = alpha / (1.0 + dist_sq)**beta
            
            # Wektor różnicy prędkości
            for i in range(3):
                new_v[i] += weight * (boid.velocity[i] - self.velocity[i]) * dt
                
        self.velocity = new_v

    def update_position(self, dt):
        for i in range(3):
            self.position[i] += self.velocity[i] * dt

class SwarmManager:
    """Zarządza logiką całego roju."""
    def __init__(self):
        self.boids = []
        
    def add_boid(self, id, pos, vel):
        self.boids.append(CuckerSmaleBoid(id, pos, vel))
        
    def step(self, dt):
        # Najpierw oblicz nowe prędkości
        for boid in self.boids:
            boid.update_velocity(self.boids, dt)
        
        # Potem zaaplikuj przesunięcie
        for boid in self.boids:
            boid.update_position(dt)
