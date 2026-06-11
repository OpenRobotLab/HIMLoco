import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


class HIMEstimator(nn.Module):
    def __init__(self,
                 temporal_steps,
                 num_one_step_obs,
                 enc_hidden_dims=[128, 64, 16],
                 activation='elu',
                 learning_rate=1e-3,
                 max_grad_norm=10.0,
                 kl_weight=1.0,
                 **kwargs):
        if kwargs:
            print("HIMEstimator.__init__ got unexpected arguments, which will be ignored: " +
                  str([key for key in kwargs.keys()]))
        super(HIMEstimator, self).__init__()
        activation_fn = get_activation(activation)

        self.temporal_steps = temporal_steps
        self.num_one_step_obs = num_one_step_obs
        self.num_latent = enc_hidden_dims[-1]
        self.max_grad_norm = max_grad_norm
        self.kl_weight = kl_weight

        # Encoder: outputs vel(3) + mu(num_latent) + logvar(num_latent)
        enc_input_dim = self.temporal_steps * self.num_one_step_obs
        enc_layers = []
        for l in range(len(enc_hidden_dims) - 1):
            enc_layers += [nn.Linear(enc_input_dim, enc_hidden_dims[l]), activation_fn]
            enc_input_dim = enc_hidden_dims[l]
        enc_layers += [nn.Linear(enc_input_dim, 3 + self.num_latent * 2)]
        self.encoder = nn.Sequential(*enc_layers)

        self.learning_rate = learning_rate
        self.optimizer = optim.Adam(self.parameters(), lr=self.learning_rate)

    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def get_latent(self, obs_history):
        """Inference: use mu directly (no sampling noise)."""
        out = self.encoder(obs_history.detach())
        vel = out[..., :3]
        mu  = out[..., 3:3 + self.num_latent]
        return vel.detach(), mu.detach()

    def forward(self, obs_history):
        return self.get_latent(obs_history)

    def encode(self, obs_history):
        """Training: sample z via reparameterization."""
        out = self.encoder(obs_history.detach())
        vel    = out[..., :3]
        mu     = out[..., 3:3 + self.num_latent]
        logvar = out[..., 3 + self.num_latent:]
        z = self.reparameterize(mu, logvar)
        return vel, mu, logvar, z

    def update(self, obs_history, next_critic_obs, lr=None):
        if lr is not None:
            self.learning_rate = lr
            for param_group in self.optimizer.param_groups:
                param_group['lr'] = self.learning_rate

        # Ground-truth velocity from privileged obs
        vel_gt = next_critic_obs[:, self.num_one_step_obs:self.num_one_step_obs + 3].detach()

        pred_vel, mu, logvar, _ = self.encode(obs_history)

        estimation_loss = F.mse_loss(pred_vel, vel_gt)

        # KL divergence: D_KL( N(mu, sigma) || N(0,1) )
        kl_loss = -0.5 * torch.mean(1 + logvar - mu.pow(2) - logvar.exp())

        loss = estimation_loss + self.kl_weight * kl_loss

        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.parameters(), self.max_grad_norm)
        self.optimizer.step()

        return estimation_loss.item(), kl_loss.item()


def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.ReLU()
    elif act_name == "silu":
        return nn.SiLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None

        
