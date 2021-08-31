import numpy as np

def matching(cost_mat, numWays=1):
    assignment = np.full(cost_mat.shape, False)
    cost = 0

    cost_mat[cost_mat!=cost_mat]=float("inf")
    validMat = cost_mat < float("inf")
    validRow = np.any(validMat, axis=1)
    validCol = np.any(validMat, axis=0)
    
    nCols = sum(validCol)
    nRows = sum(validRow)
    nTasks = int(nCols/numWays)
    
    if nRows > nTasks:
        nAdd = np.zeros((nRows,(nRows-nTasks)*numWays))
        cost_mat= np.hstack((cost_mat, nAdd))
    
    elif nRows < nTasks:
        nAdd = np.zeros(((nTasks-nRows),numWays*nTasks))
        cost_mat= np.vstack((cost_mat, nAdd))
    
    nRows_tp, nCols_tp = cost_mat.shape
    dMat = np.delete(cost_mat, ~validRow, axis=0)
    dMat = np.delete(dMat, ~validCol, axis=1)
    
    #########################################################
    #   STEP 1: Subtract the row minimum from each row.
    #########################################################
    
    for i in range(nRows):
        dMat[i,:]-=min(dMat[i,:])
    #Find the minimum value of each row
    
    ###########################################################################  
    #   STEP 2: Find a zero of dMat. If there are no starred zeros in its
    #           column or row start the zero. Repeat for each zero
    ###########################################################################
    
    s, t = np.where(dMat==0)
    zP = np.full(dMat.shape, False)
    zP[s,t] = True
    starZ = np.full((nRows, nCols), False)
    
    while np.any(zP):
        r, c = np.where(zP>0)
        hey=dMat.shape[0]*dMat.shape[1]
        yeye=len(r)
        for i in range(len(r)):
            who=dMat.shape[0]*c[i]+r[i]
            if who<hey:
                hey=who
                yeye=i
            
        starZ[r[i], c[i]] = True 
        zP[r[i],:] = False
        k = int(np.ceil((c[i]+1)/numWays))-1
        zP[:,k*numWays:(k+1)*numWays] = False

    while True:
        
        ###########################################################################
        #   STEP 3: Cover each column with a starred zero. If all the columns are
        #           covered then the matching is maximum
        # If all the columns have zero value, then there is nothing to do more.
        # If some of the columns do not have zeros, then goto step4.
        ###########################################################################
        
        primeZ = np.full((nRows_tp, nCols_tp), False)
        coverColumn_tp = np.any(starZ, axis=0)
        coverTask = np.full(nTasks, False)
        k = np.ceil((np.where(coverColumn_tp>0)[0]+1)/numWays).astype(np.int)-1
        coverTask[k]=True
        coverColumn_tp = coverTask
        for i in range(0, nTasks):
            for j in range(1, numWays):
                coverColumn_tp=np.insert(coverColumn_tp, i*numWays+j, coverTask[i])
    
        if ~np.any(~coverColumn_tp):
            break
        
        coverRow = np.full(nRows, False)

        while True:
            
            #**************************************************************************
            #   STEP 4: Find a noncovered zero and prime it.  If there is no starred
            #           zero in the row containing this primed zero, Go to Step 5.  
            #           Otherwise, cover this row and uncover the column containing 
            #           the starred zero. Continue in this manner until there are no 
            #           uncovered zeros left. Save the smallest uncovered value and 
            #           Go to Step 6.
            # Covering step, i.e. drawing the covering lines step
            #**************************************************************************
            
            zP[:,:]=False
            _dMat = ~dMat.astype('bool')
            zP[~coverRow.reshape(nRows,1)*~coverColumn_tp] = _dMat[~coverRow.reshape(nRows,1)*~coverColumn_tp]
            Step = 6
            
            while np.any(np.any(zP[~coverRow.reshape(nRows,1)*~coverColumn_tp])):
                uZr = np.where(zP>0)[0][0]
                uZc = np.where(zP>0)[1][0]
                primeZ[uZr, uZc] = True
                stz = starZ[uZr, :]
                
                if np.any(stz):
                    stz_tp=np.full(nCols, False)
                    k = int(np.ceil((np.where(stz>0)[0]+1)/numWays).astype(np.int)*numWays-1)
                    stz_tp[k-numWays+1:k+1]=True
                
                if ~np.any(stz):
                    Step=5
                    break
                
                coverRow[uZr] = True
                coverColumn_tp[stz_tp] = False;
                zP[uZr, :]=False
                a = np.where(coverRow==False)[0]
                zP[~coverRow.reshape(nRows,1)*stz]=_dMat[~coverRow.reshape(nRows,1)*stz]
            
            if Step == 6:
                
                # *************************************************************************
                # STEP 6: Add the minimum uncovered value to every element of each covered
                #         row, and subtract it from every element of each uncovered column.
                #         Return to Step 4 without altering any stars, primes, or covered lines.
                # If there is no matching (since one or more column or row have
                # no assignment yet), subtract the rest minimum value from the
                # uncovered elements. (Add it to crossed values)
                #**************************************************************************
               
                M=dMat[~coverRow.reshape(nRows,1)*~coverColumn_tp].reshape(sum(~coverRow),sum(~coverColumn_tp))
                if ((M.size==0) and ((nCols/numWays)>nRows)):
                    assignment[validRow.reshape(nRows_tp,1)*validCol]=starZ[:nRows,:nCols].reshape(nRows*nCols)
                    cost = sum(cost_mat[assignment])
                    return assignment, cost
                
                minval=np.min(M)
                
                if minval==float("inf"):
                    return assignment, cost
                
                dMat[coverRow.reshape(nRows,1)*coverColumn_tp]=dMat[coverRow.reshape(nRows,1)*coverColumn_tp]+minval
                dMat[~coverRow.reshape(nRows,1)*~coverColumn_tp]=M.reshape(sum(~coverRow)*sum(~coverColumn_tp))-minval

            else: 
                break

        #**************************************************************************
        # STEP 5:
        #  Construct a series of alternating primed and starred zeros as
        #  follows:
        #  Let Z0 represent the uncovered primed zero found in Step 4.
        #  Let Z1 denote the starred zero in the column of Z0 (if any).
        #  Let Z2 denote the primed zero in the row of Z1 (there will always
        #  be one).  Continue until the series terminates at a primed zero
        #  that has no starred zero in its column.  Unstar each starred
        #  zero of the series, star each primed zero of the series, erase
        #  all primes and uncover every line in the matrix.  Return to Step 3.
        #  Finding assignment step (choosing elements)
        #**************************************************************************
        
        uZc_tp = np.full(nCols, False)
        k = int(np.ceil((uZc+1)/numWays).astype(np.int)*numWays-1)
        uZc_tp[k-numWays+1:k+1]=True
        rowZ1 = starZ[:, uZc_tp]
        
        if rowZ1.shape[1]>1:
            rowZ1 = np.any(rowZ1, axis=1).reshape(nRows,1)
            
        starZ[uZr, uZc] = True
        
        while np.any(rowZ1):
            t4 = np.full(nCols_tp, True)
            t2 = np.full((nRows,1), True)
            starZ[rowZ1*uZc_tp]=False
            uZc = primeZ[rowZ1*t4]
            uZc_tp = np.full(nCols, False)
            k = int(np.ceil((np.where(uZc)[0]+1)/numWays).astype(np.int)*numWays-1)
            uZc_tp[k-numWays+1:k+1]=True
            uZr=rowZ1
            rowZ1=starZ[t2*uZc_tp].reshape(int(sum(t2)),sum(uZc_tp))
            
            if rowZ1.shape[1]>1:
                rowZ1 = np.any(rowZ1, axis=1).reshape(nRows,1)
                
            starZ[uZr*uZc]=True
            
    assignment[validRow.reshape(nRows_tp,1)*validCol]=starZ[:nRows,:nCols].reshape(nRows*nCols)
    cost = sum(cost_mat[assignment])
    
    return assignment, cost