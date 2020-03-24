import gym_env
import numpy as np
import time
import tensorflow.compat.v1 as tf
import copy
import os
import pickle as pkl

folder = os.getcwd() + '/../model/rdpg'
model_name = '/subt_rl495.ckpt-1037'
record_name = 'straight.pkl'
input_checkpoint = folder+model_name

saver = tf.train.import_meta_graph(
    input_checkpoint + '.meta', clear_devices=True)
graph = tf.get_default_graph()
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
saver.restore(sess, input_checkpoint)

##########  get tensor ##########
input_s = graph.get_tensor_by_name('s:0')

output_name = 'Actor/eval/action_out:0'
output_a = graph.get_tensor_by_name(output_name)

state_name0 = 'Actor/eval/LSTMCellZeroState/zeros:0'
lstm_state0 = graph.get_tensor_by_name(state_name0)
state_name1 = 'Actor/eval/LSTMCellZeroState/zeros_1:0'
lstm_state1 = graph.get_tensor_by_name(state_name1)
lstm_state_in = (lstm_state0, lstm_state1)

out_name0 = 'Actor/eval/rnn/while/Exit_3:0'
out_state0 = graph.get_tensor_by_name(out_name0)
out_name1 = 'Actor/eval/rnn/while/Exit_4:0'
out_state1 = graph.get_tensor_by_name(out_name1)
lstm_state_out = (out_name0, out_name1)
hidden_state = (np.zeros([1,128]),np.zeros([1,128]))

##########  get tensor ##########

# evaluation
env = gym_env.SubtCaveNobackEnv()

env.set_max_dis(5)
s = env.reset()

epoch = 0
iteration = 60
record = np.zeros([iteration, 2])

for i in range(iteration):
    start_time = time.time()
    ep_reward = 0
    step = 0
    distance = 0
    while True:
        s = s[np.newaxis, :]
        # actions, hidden_state = sess.run([output_a, lstm_state_out], {
        #                                     input_s: s, lstm_state_in: hidden_state})
        # action_out = actions[0]
        action_out = [1,0]
        s_, r, done, info = env.step(action_out)
        s = s_
        step += 1
        if done or env.total_dis > 600:
            record[epoch][0] = env.total_dis
            record[epoch][1] = time.time()-start_time
            s = env.reset()
            break
    print epoch, record[epoch]
    hidden_state = (np.zeros([1,128]),np.zeros([1,128]))

    epoch += 1
    # print(record)

fileObject = open(folder+"/"+record_name, 'wb')

pkl.dump(record, fileObject)
fileObject.close()
