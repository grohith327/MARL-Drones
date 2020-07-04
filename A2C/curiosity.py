import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class FeatureExtractor(nn.Module):
    def __init__(self, obs_size, out_size):
        super().__init__()
        self.dense1 = nn.Linear(obs_size, 128)
        self.dense2 = nn.Linear(128, 64)
        self.dense3 = nn.Linear(64, out_size)

    def forward(self, obs):
        out = F.leaky_relu(self.dense1(obs))
        out = F.leaky_relu(self.dense2(out))
        out = self.dense3(out)
        return out


class InverseModel(nn.Module):
    def __init__(self, input_size, action_space):
        super().__init__()
        self.in1 = nn.Linear(input_size, input_size)
        self.in2 = nn.Linear(input_size, input_size)
        self.dense1 = nn.Linear(input_size * 2, 32)
        self.dense2 = nn.Linear(32, 16)
        self.dense3 = nn.Linear(16, action_space)

    def forward(self, x1, x2):
        x1 = F.leaky_relu(self.in1(x1))
        x2 = F.leaky_relu(self.in2(x2))
        out = torch.cat((x1, x2), dim=-1)
        out = F.leaky_relu(self.dense1(out))
        out = F.leaky_relu(self.dense2(out))
        out = self.dense3(out)
        return out


class ForwardModel(nn.Module):
    def __init__(self, input_size, action_space):
        super().__init__()
        self.act_in1 = nn.Linear(action_space, 16)
        self.act_in2 = nn.Linear(16, input_size)
        self.state_in1 = nn.Linear(input_size, input_size)
        self.dense1 = nn.Linear(input_size * 2, input_size)

    def forward(self, act, state):
        act = F.leaky_relu(self.act_in1(act))
        act = F.leaky_relu(self.act_in2(act))
        state = F.leaky_relu(self.state_in1(state))
        out = torch.cat((state, act), dim=-1)
        out = self.dense1(out)
        return out


class ICM:
    def __init__(self, obs_size, action_space, hidden_size=32):
        self.features = FeatureExtractor(obs_size, hidden_size)
        self.inverse_model = InverseModel(hidden_size, action_space)
        self.forward_model = ForwardModel(hidden_size, action_space)

    def class_loss(self, pred, target):
        loss_fct = nn.CrossEntropyLoss()
        loss = loss_fct(pred, target)
        return loss

    def l2_loss(self, pred, target):
        loss_fct = nn.MSELoss()
        loss = loss_fct(pred, target)
        return loss

    def __call__(self, a_t, a_t_logits, s_t, s_t1):
        phi_st = self.features(s_t)
        phi_st1 = self.features(s_t1)
        pred_at = self.inverse_model(phi_st, phi_st1)
        loss = self.class_loss(pred_at, a_t)
        loss.backward(retain_graph=True)

        pred_phi_st1 = self.forward_model(a_t_logits, phi_st)
        loss_fm = self.l2_loss(pred_phi_st1, phi_st1) * 0.5
        r_i = loss_fm.item()
        loss_fm.backward()

        r_i *= 10
        return r_i
