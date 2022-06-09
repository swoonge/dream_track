#!/usr/bin/env python3
# -- coding: utf-8 --
import numpy as np
from matplotlib import pyplot as plt

class DBSCAN():
    def __init__(self, epsilon, minpts):
        self.epsilon = epsilon
        self.minpts = minpts

    #idx를 만드는 함수
    def clustering(self):
        # Clustering
        for i in range(len(self.input)):
            if self.visited[i] == False:
                self.visited[i] = True
                self.neighbors = self.regionQuery(i)
                if len(self.neighbors) >= self.minpts:
                    self.C += 1
                    self.expandCluster(i)
                else:
                    self.noise[i] = True

    def regionQuery(self, i):
        g = self.dist[i,:] < self.epsilon
        Neighbors = np.where(g)[0].tolist()
        return Neighbors

    def expandCluster(self, i):
        self.idx[i] = self.C
        k = 0
       
        while True:
            if len(self.neighbors) <= k:return
            j = self.neighbors[k]
            if self.visited[j] != True:
                self.visited[j] = True

                self.neighbors2 = self.regionQuery(j)
                v = [self.neighbors2[i] for i in np.where(self.idx[self.neighbors2]==0)[0]]

                if len(self.neighbors2) >=  self.minpts:
                    self.neighbors = self.neighbors+v

            if self.idx[j] == 0 : self.idx[j] = self.C
            k += 1

    # cluster와 noise를 만드는 함수
    def sort(self):
        cnum = np.max(self.idx)
        self.cluster = []
        self.noise = []
        for i in range(cnum):
            k = np.where(self.idx == (i+1))[0].tolist()
            self.cluster.append([self.input[k,:]])
       
        self.noise = self.input[np.where(self.idx == 0)[0].tolist(),:]

    def plot(self):
        fig,ax = plt.subplots()
        
        try:
            for idx,group in enumerate(self.cluster):
                ax.plot(group[0][:,0],
                        group[0][:,1],
                        marker='o',
                        linestyle='',
                        label='Cluster {}'.format(idx))
                ax.plot(self.cluster_avg[idx][0],
                        self.cluster_avg[idx][1],
                        marker='*',
                        linestyle='',
                        label='Cluster {}'.format(idx))

            if self.noise.size != 0:
                ax.plot(self.noise[:,0],
                        self.noise[:,1],
                        marker='x',
                        linestyle='',
                        label='noise')
        except:
            pass

        ax.legend(fontsize=10, loc='upper left')
        plt.title('Scatter Plot of Clustering result', fontsize=15)
        plt.xlabel('X', fontsize=14)
        plt.ylabel('Y', fontsize=14)
        plt.show()

    def plot_sign(self, xy):
        fig,ax = plt.subplots()
        
        for idx,group in enumerate(xy):
            ax.plot(group[0][:,0],
                    group[0][:,1],
                    marker='o',
                    linestyle='',
                    label='sign {}'.format(idx))

        ax.legend(fontsize=10, loc='upper left')
        plt.title('Scatter Plot of Clustering result', fontsize=15)
        plt.xlabel('X', fontsize=14)
        plt.ylabel('Y', fontsize=14)
        plt.show()

    def data_update(self, x):
        try:
            self.n = len(x)
            #Euclidean 
            p, q = np.meshgrid(np.arange(self.n), np.arange(self.n))
            self.dist = np.sqrt(np.sum(((x[p] - x[q])**2),2))
        except:
            x = np.array(x)
            self.n = len(x)
            p, q = np.meshgrid(np.arange(self.n), np.arange(self.n))
            self.dist = np.sqrt(np.sum(((x[p] - x[q])**2),2))
        # label as visited points and noise
        self.visited = np.full((self.n), False)
        self.noise = np.full((self.n),False)
        # Cluseter
        self.idx = np.full((self.n),0)
        self.C = 0
        self.input = x

    def get_cluster_avg(self):
        avg = list()
        for c in self.cluster:
            avg.append(np.mean(np.array(c[0]), axis = 0).tolist())
        self.cluster_avg = avg

    def run(self, x):
        self.data_update(x) # np 2channal array input
        # DBSCAN Parameters

        self.clustering()
        self.sort()
        self.get_cluster_avg()

        return len(self.cluster)
