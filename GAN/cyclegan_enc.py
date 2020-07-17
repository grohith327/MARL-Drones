from simplegan.gan import CycleGAN
from simplegan.datasets import cyclegan_dataloader
import os
from tqdm.auto import tqdm
import tensorflow as tf
import segmentation_models as sm

sm.set_framework("tf.keras")


class ModifiedGAN(CycleGAN):
    def __init__(self, BACKBONE):
        CycleGAN.__init__(self)
        self.backbone = BACKBONE

    def load_data(self, data_dir, img_width=224, img_height=224, batch_size=16):
        data_obj = cyclegan_dataloader(
            datadir=data_dir, img_width=img_width, img_height=img_height
        )
        trainA, trainB, testA, testB = data_obj.load_dataset()

        for data in trainA.take(1):
            self.img_size = data.shape
            self.channels = data.shape[-1]

        trainA = trainA.shuffle(100000).batch(batch_size)
        trainB = trainB.shuffle(100000).batch(batch_size)

        testA = testA.shuffle(100000).batch(batch_size)
        testB = testB.shuffle(100000).batch(batch_size)

        return trainA, trainB, testA, testB

    def generator(self):
        model = sm.Unet(
            self.backbone,
            encoder_weights="imagenet",
            input_shape=self.img_size,
            classes=3,
        )
        return model


gan = ModifiedGAN(BACKBONE="efficientnetb5")
trainA, trainB, testA, testB = gan.load_data(data_dir="./data", batch_size=4)
gan.fit(
    trainA,
    trainB,
    testA,
    testB,
    epochs=8,
    gen_g_learning_rate=0.001,
    gen_f_learning_rate=0.001,
    disc_x_learning_rate=0.00001,
    disc_y_learning_rate=0.00001,
    LAMBDA=1,
    tensorboard=True,
    save_model="./",
)
gan.generate_samples(testA.shuffle(10000).take(20), save_dir="./generated_samples")
