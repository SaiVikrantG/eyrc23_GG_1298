'''
*****************************************************************************************
*
*        		===============================================
*           		GeoGuide(GG) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 1A of GeoGuide(GG) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ 1289 ]
# Author List:		[ Anoshor B Paul, G Sai Vikrant, Pradyumna V Acharya, Pritham Suvarna ]
# Filename:			task_1a.py
# Functions:	    [`ideantify_features_and_targets`, `load_as_tensors`,
# 					 `model_loss_function`, `model_optimizer`, `model_number_of_epochs`, `training_function`,
# 					 `validation_functions` ]

####################### IMPORT MODULES #######################
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
import torch
from torch.utils.data import TensorDataset, DataLoader
import torch.nn as nn
import torch.nn.functional as F
from sklearn.model_selection import train_test_split
##############################################################

def data_preprocessing(task_1a_dataframe):

    ''' 
	Purpose:
	---
	This function will be used to load your csv dataset and preprocess it.
	Preprocessing involves cleaning the dataset by removing unwanted features,
	decision about what needs to be done with missing values etc. Note that 
	there are features in the csv file whose values are textual (eg: Industry, 
	Education Level etc)These features might be required for training the model
	but can not be given directly as strings for training. Hence this function 
	should return encoded dataframe in which all the textual features are 
	numerically labeled.
	
	Input Arguments:
	---
	`task_1a_dataframe`: [Dataframe]
						  Pandas dataframe read from the provided dataset 	
	
	Returns:
	---
	`encoded_dataframe` : [ Dataframe ]
						  Pandas dataframe that has all the features mapped to 
						  numbers starting from zero

	Example call:
	---
	encoded_dataframe = data_preprocessing(task_1a_dataframe)
	'''

    #################	ADD YOUR CODE HERE	##################
    df = task_1a_dataframe

    
    df['Education'].replace({"Bachelors":0,"Masters":1,"PHD":2},inplace=True)
    df['Gender'].replace({"Male":0,"Female":1},inplace=True)
    df['EverBenched'].replace({"No":0,"Yes":1},inplace=True)
    MinMx = MinMaxScaler()
    Scaled_columns = ['Age', 'JoiningYear','ExperienceInCurrentDomain']
    df[Scaled_columns] = MinMx.fit_transform(df[Scaled_columns])
    df = pd.get_dummies(columns=['City'],data=df,dtype=int)
	

    
    # Display the encoded DataFrame
    #df_encoded = df_encoded.astype(int)
    
    encoded_dataframe = df
    ##########################################################

    return encoded_dataframe


def identify_features_and_targets(encoded_dataframe):
    '''
	Purpose:
	---
	The purpose of this function is to define the features and
	the required target labels. The function returns a python list
	in which the first item is the selected features and second 
	item is the target label

	Input Arguments:
	---
	`encoded_dataframe` : [ Dataframe ]
						Pandas dataframe that has all the features mapped to 
						numbers starting from zero
	
	Returns:
	---
	`features_and_targets` : [ list ]
							python list in which the first item is the 
							selected features and second item is the target label

	Example call:
	---
	features_and_targets = identify_features_and_targets(encoded_dataframe)
	'''

    #################	ADD YOUR CODE HERE	##################
    features = encoded_dataframe.drop(columns=['LeaveOrNot'])
    target = encoded_dataframe['LeaveOrNot']

    features_and_targets = [features, target]		

    ##########################################################
    

    return features_and_targets


def load_as_tensors(features_and_targets):
    
    ''' 
	Purpose:
	---
	This function aims at loading your data (both training and validation)
	as PyTorch tensors. Here you will have to split the dataset for training 
	and validation, and then load them as as tensors. 
	Training of the model requires iterating over the training tensors. 
	Hence the training sensors need to be converted to iterable dataset
	object.
	
	Input Arguments:
	---
	`features_and targets` : [ list ]
							python list in which the first item is the 
							selected features and second item is the target label
	
	Returns:
	---
	`tensors_and_iterable_training_data` : [ list ]
											Items:
											[0]: X_train_tensor: Training features loaded into Pytorch array
											[1]: X_test_tensor: Feature tensors in validation data
											[2]: y_train_tensor: Training labels as Pytorch tensor
											[3]: y_test_tensor: Target labels as tensor in validation data
											[4]: Iterable dataset object and iterating over it in 
												 batches, which are then fed into the model for processing

	Example call:
	---
	tensors_and_iterable_training_data = load_as_tensors(features_and_targets)
	'''
    #################	ADD YOUR CODE HERE	##################
    [X, y] = features_and_targets
    X = X.values
    y = y.values
    
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=0)


    X_train_tensor = torch.FloatTensor(X_train)
    X_test_tensor = torch.FloatTensor(X_test)
    y_train_tensor = torch.LongTensor(y_train)
    y_test_tensor = torch.LongTensor(y_test)
    
    train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
    batch_size = 165 # You can adjust the batch size as needed
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    
    tensors_and_iterable_training_data = [X_train_tensor, X_test_tensor, y_train_tensor, y_test_tensor, train_loader]
    ##########################################################

    return tensors_and_iterable_training_data

class Salary_Predictor(nn.Module):
    '''
	Purpose:
	---
	The architecture and behavior of your neural network model will be
	defined within this class that inherits from nn.Module. Here you
	also need to specify how the input data is processed through the layers. 
	It defines the sequence of operations that transform the input data into 
	the predicted output. When an instance of this class is created and data
	is passed through it, the `forward` method is automatically called, and 
	the output is the prediction of the model based on the input data.
	
	Returns:
	---
	`predicted_output` : Predicted output for the given input data
	'''
    def __init__(self, input_features=10, hidden1=80, hidden2=80, output_features=1):
        super(Salary_Predictor, self).__init__()
        self.fc1 = nn.Linear(input_features, hidden1)
        self.fc2 = nn.Linear(hidden1, hidden2)
        self.out = nn.Linear(hidden2, output_features)

    def forward(self, x):
        #x = x.requires_grad_()
        x = F.leaky_relu(self.fc1(x))
        x = F.leaky_relu(self.fc2(x))
        '''x = F.sigmoid(self.out(x))
        x = (x >= 0.5).float().requires_grad_()'''
        x = self.out(x)
        #x = x.long()

        return x

def model_loss_function():
    '''
	Purpose:
	---
	To define the loss function for the model. Loss function measures 
	how well the predictions of a model match the actual target values 
	in training data.
	
	Input Arguments:
	---
	None

	Returns:
	---
	`loss_function`: This can be a pre-defined loss function in PyTorch
					or can be user-defined

	Example call:
	---
	loss_function = model_loss_function()
	'''
    loss_function = nn.MSELoss()
    
    return loss_function



def model_optimizer(model):
    '''
	Purpose:
	---
	To define the optimizer for the model. Optimizer is responsible 
	for updating the parameters (weights and biases) in a way that 
	minimizes the loss function.
	
	Input Arguments:
	---
	`model`: An object of the 'Salary_Predictor' class

	Returns:
	---
	`optimizer`: Pre-defined optimizer from Pytorch

	Example call:
	---
	optimizer = model_optimizer(model)
	'''
    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
    
    return optimizer

def model_number_of_epochs():
    '''
	Purpose:
	---
	To define the number of epochs for training the model

	Input Arguments:
	---
	None

	Returns:
	---
	`number_of_epochs`: [integer value]

	Example call:
	---
	number_of_epochs = model_number_of_epochs()
	'''
    number_of_epochs=20
    
    return number_of_epochs


def training_function(model, number_of_epochs, tensors_and_iterable_training_data, loss_function, optimizer):
    '''
	Purpose:
	---
	All the required parameters for training are passed to this function.

	Input Arguments:
	---
	1. `model`: An object of the 'Salary_Predictor' class
	2. `number_of_epochs`: For training the model
	3. `tensors_and_iterable_training_data`: list containing training and validation data tensors 
											 and iterable dataset object of training tensors
	4. `loss_function`: Loss function defined for the model
	5. `optimizer`: Optimizer defined for the model

	Returns:
	---
	trained_model

	Example call:
	---
	trained_model = training_function(model, number_of_epochs, iterable_training_data, loss_function, optimizer)

	'''
    training_data = TensorDataset(tensors_and_iterable_training_data[0],tensors_and_iterable_training_data[2])
    training_loader = tensors_and_iterable_training_data[4]
    #training_loader = DataLoader(training_data, batch_size=64, shuffle=True)

    #validation_loader = torch.utils.data.DataLoader(validation_data, batch_size=64, shuffle=True)
    for epoch in range(number_of_epochs):
        running_loss = 0.0
        for i, data in enumerate(training_loader, 0):
            # get the inputs; data is a list of [inputs, labels]
            inputs, labels = data
            # print(i)
            # zero the parameter gradients
            optimizer.zero_grad()
            labels = labels.unsqueeze(1)
            
            # forward + backward + optimize 
            outputs = model(inputs)
            
            # print(outputs)
            loss = loss_function(outputs.float(), labels.float())
            # Backward pass
            loss.backward()
            # Update model parameters
            optimizer.step()

            # print statistics
            running_loss += loss.item()
        average_loss = running_loss / len(training_loader)
        
        # Print epoch-wise loss
        #print(f"Epoch {epoch+1}/{number_of_epochs}: Loss = {average_loss}")
            # if i % 2000 == 1999:    # print every 2000 mini-batches
            #     print('[%d, %5d] loss: %.3f' %
            #           (epoch + 1, i + 1, running_loss / 2000))
            #     running_loss = 0.0

           
    return model

def validation_function(trained_model, tensors_and_iterable_training_data):
    '''
	Purpose:
	---
	This function will utilise the trained model to do predictions on the
	validation dataset. This will enable us to understand the accuracy of
	the model.

	Input Arguments:
	---
	1. `trained_model`: Returned from the training function
	2. `tensors_and_iterable_training_data`: list containing training and validation data tensors 
											 and iterable dataset object of training tensors

	Returns:
	---
	model_accuracy: Accuracy on the validation dataset

	Example call:
	---
	model_accuracy = validation_function(trained_model, tensors_and_iterable_training_data)

	'''

    # Set the model to evaluation mode
    trained_model.eval()
    
    # Unpack the input arguments
    validation_data = tensors_and_iterable_training_data[1]
    validation_labels = tensors_and_iterable_training_data[3]
    # Calculate the predictions of the trained model on the validation data
    with torch.no_grad():
        predictions = trained_model(validation_data)
        
    # Convert the predicted probabilities to class labels
    _, predicted_labels = torch.max(predictions, 1)
    #print(predicted_labels)
    #print(validation_labels)
    #print(predicted_labels==validation_labels)

    # Calculate the accuracy of the model
    correct_predictions = (predicted_labels == validation_labels).sum().item()
    total_predictions = len(validation_labels)
    model_accuracy = correct_predictions / total_predictions

    
    return model_accuracy

########################################################################
########################################################################
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########	
'''
	Purpose:
	---
	The following is the main function combining all the functions
	mentioned above. Go through this function to understand the flow
	of the script

'''

if __name__ == "__main__":

	# reading the provided dataset csv file using pandas library and 
	# converting it to a pandas Dataframe
	task_1a_dataframe = pd.read_csv('task_1a_dataset.csv')

	# data preprocessing and obtaining encoded data
	encoded_dataframe = data_preprocessing(task_1a_dataframe)
    
	# selecting required features and targets
	features_and_targets = identify_features_and_targets(encoded_dataframe)

	# obtaining training and validation data tensors and the iterable
	# training data object
	tensors_and_iterable_training_data = load_as_tensors(features_and_targets)
	
	# model is an instance of the class that defines the architecture of the model
	model = Salary_Predictor()

	# obtaining loss function, optimizer and the number of training epochs
	loss_function = model_loss_function()
	optimizer = model_optimizer(model)
	number_of_epochs = model_number_of_epochs()

	# training the model
	trained_model = training_function(model, number_of_epochs, tensors_and_iterable_training_data, 
					loss_function, optimizer)

	# validating and obtaining accuracy
	model_accuracy = validation_function(trained_model,tensors_and_iterable_training_data)
	print(f"Accuracy on the test set = {model_accuracy}")

	X_train_tensor = tensors_and_iterable_training_data[0]
	x = X_train_tensor[0]
	jitted_model = torch.jit.save(torch.jit.trace(model, (x)), "task_1a_trained_model.pth")

