from feature_extraction import extract_features
import glob
import numpy as np
import os.path
import matplotlib.image as mpimg
import pickle
from PIL import Image
import random
from sklearn.externals import joblib
from sklearn.model_selection import GridSearchCV
from sklearn.metrics import accuracy_score
from sklearn.metrics import f1_score
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import LinearSVC
from sklearn.utils import shuffle

# TODO: train SVM on bias instead of accuracy to favor false positives

# returns all images in a given directory
def import_images(directory, n_files='ALL', shuffle=False):
    images = []
    files = glob.glob(directory + '/*.jpg')
    if shuffle:
        random.shuffle(files)
    else:
        files.sort()
    if n_files == 'ALL' or n_files > len(files):
        n_files = len(files)
    for i, filename in enumerate(files):
        if i == n_files:
            break
        img = Image.open(filename)
        images.append(np.array(img))
    return images

X = []
n_files = 80000

X += import_images(directory='positives/yellow', n_files=n_files, shuffle=True)
print("Imported yellow buoys")

X += import_images(directory='positives/red', n_files=n_files, shuffle=True)
print("Imported red buoys")

X += import_images(directory='positives/green', n_files=n_files, shuffle=True)
print("Imported green buoys")

y = [1 for i in range(len(X))]

l = len(X)
X += import_images('negatives', n_files=3*n_files, shuffle=True)
print("Imported negative examples")
y += [0 for i in range(len(X) - l)]

X, y = shuffle(X, y, random_state=1)

# can be RGB, HSV, LUV, HLS, YUV, YCrCb
color_space = 'YCrCb'
spatial_size = (32, 32)
hist_bins = 32
orient = 9
pix_per_cell = 8
cell_per_block = 2
# can be 0, 1, 2, or 'ALL'
hog_channel = 0
spatial_feat = True
hist_feat = False
hog_feat = True

# extract features
X_feat = extract_features(X, color_space=color_space, spatial_size=spatial_size, hist_bins=hist_bins,
                         orient=orient, pix_per_cell=pix_per_cell, cell_per_block=cell_per_block,
                         hog_channel=hog_channel, spatial_feat=spatial_feat, hist_feat=hist_feat,
                         hog_feat=hog_feat)
del X
print("Extracted features")

# shuffle data
X_feat, y = shuffle(X_feat, y, random_state=1)
print("Shuffled data")

# split data into 60:20:20 training, validation, and test sets

# tune SVC parameters on fifth of data
fraction_of_data = 3

X_train, X_test, y_train, y_test = train_test_split(X_feat[:int(len(X_feat)/fraction_of_data)], y[:int(len(y)/fraction_of_data)], test_size=0.2, random_state=2)
X_train_new, X_val, y_train_new, y_val = train_test_split(X_train, y_train, test_size=0.25, random_state=3)
print("Created validation and test sets")

# standardize features to 0 mean and unit variance
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train_new)
del X_train_new
X_val_scaled = scaler.transform(X_val)
del X_val
print("Scaled data")

# tune C on validation set
C_grid = [pow(2, i) for i in range(-20, 21)]
best_c, best_score = 1, 0

for i, c in enumerate(C_grid):
    svc = LinearSVC(random_state=3, C=c)
    svc.fit(X_train_scaled, y_train_new)
    y_pred = svc.predict(X_val_scaled)
    score = accuracy_score(y_pred, y_val)
    # score = f1_score(y_pred, y_val)
    print("Count: {} / {}".format(i + 1, len(C_grid)))
    print("Score: {}\n".format(score))
    if score > best_score:
        best_score = score
        best_c = c

print("\nBest C: {}".format(best_c))
print("Best score: {}".format(best_score))

del svc, X_val_scaled, y_pred, y_val

# rescale data to entire training set
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
del X_train
X_test_scaled = scaler.transform(X_test)
del scaler, X_test
print("Scaled data")

# test SVC on test set
svc = LinearSVC(random_state=3, C=best_c)
svc.fit(X_train_scaled, y_train)
del X_train_scaled, y_train
y_pred = svc.predict(X_test_scaled)
del X_test_scaled
score = accuracy_score(y_pred, y_test)
# score = f1_score(y_pred, y_test)
del y_pred, y_test
print("Accuracy score on test set: {}".format(score))

# fit scaler
scaler = StandardScaler()
X_feat_scaled = scaler.fit_transform(X_feat)
print("Fit scaler")
del X_feat

# pickle scaler for later use
joblib.dump(scaler, 'scaler5.pkl')
del scaler

# fit svc
svc = LinearSVC(random_state=3, C=best_c)
svc.fit(X_feat_scaled, y)
del X_feat_scaled, y
print("Fit SVM")

# pickle classifier for later use
joblib.dump(svc, 'svc5.pkl')