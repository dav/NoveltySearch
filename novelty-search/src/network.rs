use rand::Rng;

/// A simple feedforward neural network: 11 inputs → 5 hidden (tanh) → 2 outputs (tanh).
/// Used to map sensor inputs to (angular_velocity, speed).
#[derive(Clone)]
pub struct Network {
    /// Weights from input to hidden layer (5 × 11 = 55 weights)
    pub w_ih: [[f64; 11]; 5],
    /// Hidden layer biases (5)
    pub b_h: [f64; 5],
    /// Weights from hidden to output layer (2 × 5 = 10 weights)
    pub w_ho: [[f64; 5]; 2],
    /// Output layer biases (2)
    pub b_o: [f64; 2],
}

impl Network {
    /// Create a network with random weights in [-1, 1].
    pub fn random() -> Self {
        let mut rng = rand::rng();

        let mut w_ih = [[0.0; 11]; 5];
        for row in &mut w_ih {
            for w in row.iter_mut() {
                *w = rng.random_range(-1.0..1.0);
            }
        }

        let mut b_h = [0.0; 5];
        for b in &mut b_h {
            *b = rng.random_range(-1.0..1.0);
        }

        let mut w_ho = [[0.0; 5]; 2];
        for row in &mut w_ho {
            for w in row.iter_mut() {
                *w = rng.random_range(-1.0..1.0);
            }
        }

        let mut b_o = [0.0; 2];
        for b in &mut b_o {
            *b = rng.random_range(-1.0..1.0);
        }

        Network { w_ih, b_h, w_ho, b_o }
    }

    /// Forward pass: inputs (11) → outputs (2).
    /// Output 0 = angular velocity [-1, 1] (scaled later)
    /// Output 1 = speed [0, 1] (mapped from tanh's [-1,1] to [0,1])
    pub fn forward(&self, inputs: &[f64; 11]) -> (f64, f64) {
        // Hidden layer
        let mut hidden = [0.0; 5];
        for (h, (weights, bias)) in hidden.iter_mut().zip(self.w_ih.iter().zip(self.b_h.iter())) {
            let mut sum = *bias;
            for (w, x) in weights.iter().zip(inputs.iter()) {
                sum += w * x;
            }
            *h = sum.tanh();
        }

        // Output layer
        let mut output = [0.0; 2];
        for (o, (weights, bias)) in output.iter_mut().zip(self.w_ho.iter().zip(self.b_o.iter())) {
            let mut sum = *bias;
            for (w, h) in weights.iter().zip(hidden.iter()) {
                sum += w * h;
            }
            *o = sum.tanh();
        }

        let ang_vel = output[0] * 0.3;           // scale to reasonable turning rate
        let speed = (output[1] + 1.0) / 2.0 * 3.0; // map [-1,1] to [0, 3]

        (ang_vel, speed)
    }

    /// Return a mutated copy of this network.
    /// Each weight is perturbed by Gaussian noise with standard deviation `sigma`.
    pub fn mutated(&self, sigma: f64) -> Self {
        let mut child = self.clone();
        let mut rng = rand::rng();

        // Helper: Box-Muller transform to generate standard normal samples
        let mut gauss = || -> f64 {
            let u1: f64 = rng.random_range(1e-10..1.0); // avoid log(0)
            let u2: f64 = rng.random_range(0.0..std::f64::consts::TAU);
            (-2.0 * u1.ln()).sqrt() * u2.cos()
        };

        for row in &mut child.w_ih {
            for w in row.iter_mut() {
                *w += sigma * gauss();
            }
        }
        for b in &mut child.b_h {
            *b += sigma * gauss();
        }
        for row in &mut child.w_ho {
            for w in row.iter_mut() {
                *w += sigma * gauss();
            }
        }
        for b in &mut child.b_o {
            *b += sigma * gauss();
        }

        child
    }
}
