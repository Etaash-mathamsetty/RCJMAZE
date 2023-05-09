#import Robot as Rb
#import cv2

# debug_cv = True

# def pre_process_img(frame):
#     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     frame = cv2.medianBlur(frame, 5)
#     return frame

# if video.isOpened() and video1.isOpened():
#     frame = cv2.flip(video.read()[1], 1)
#     frame2 = cv2.flip(video1.read()[1], 1)
#     frame = pre_process_img(frame)
#     frame2 = pre_process_img(frame2)

# #TODO:
# Rb.SetDataValue("NRK", [0.0])

# if debug_cv and video.isOpened() and video1.isOpened():
#     cv2.imshow("frame", frame)
#     cv2.imshow("frame2", frame2)
import numpy as np
import math

resize_size = 20
pix_thresh = 125
label_txt = np.empty((0,1))
feature_txt = np.empty((0,resize_size**2))
knn = cv2.ml.KNearest_create()
feature_txt = np.loadtxt("feature.txt", np.float32)
print(feature_txt.shape)
label_txt = np.loadtxt("label.txt",np.float32).reshape((feature_txt.shape[0],1))
knn.train(feature_txt, cv2.ml.ROW_SAMPLE, label_txt)

#processed image
def label_img(frame):
    cv2.imshow("looks good?", frame)
    key = chr(cv2.waitKey(0)).lower()
    cv2.destroyWindow("looks good?")
    if(key == 'h' or key == 's' or key == 'u'):
        arr = cv2.resize(frame, (resize_size,resize_size), interpolation=cv2.INTER_AREA).reshape((1, resize_size**2))
        arr[arr > pix_thresh] = 255
        arr[arr < pix_thresh] = 0
        return np.array([[ord(key)]], dtype=np.float32), arr
    else:
        #empty tuple
        return ()


green_low_hsv = [50,80,0]
green_high_hsv = [90, 239, 231]
red_low_hsv = [120, 150, 0]
red_high_hsv = [200, 239, 231]
yellow_low_hsv = [7, 80, 0]
yellow_high_hsv = [40, 239, 231]

if True:
    camera_num = True
    
    count = 0
    
    while True:
        count += 1
        if camera_num:
            org = cv2.flip(video.read()[1], -1)
        else:
            org = cv2.flip(video1.read()[1], -1)
             
        frame = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)
        
        bounding_rect = []
        
        for i in range(3):
        
            lower = np.array([red_low_hsv, green_low_hsv, yellow_low_hsv])
            upper = np.array([red_high_hsv, green_high_hsv, yellow_high_hsv])

            
            mask = cv2.inRange(frame, lower[i], upper[i])
            _,color_contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(color_contours) > 0:
                mask = cv2.drawContours(mask, color_contours, -1, (0,255,0), 3)
                color_cont = max(color_contours, key=cv2.contourArea)
                if cv2.contourArea(color_cont) >= 400:
                    bounding_rect.append(cv2.boundingRect(color_cont))
                else:
                    bounding_rect.append([])
            else:
                bounding_rect.append([])
    #         mask2 = cv2.inRange(frame_red, lower_red2, upper_red2)
            
    #         mask = mask1 + mask2
            
            #frame_red = cvb2.bitwise_and(fra
            mask = cv2.bitwise_and(org, org, mask = mask)
            # cv2.imshow(str(i), mask)

        frame2 = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
        frame2 = cv2.convertScaleAbs(frame2, 1.5, 4)
        
        frame2 = cv2.medianBlur(frame2, 3)
        frame2 = cv2.GaussianBlur(frame2, (7,7), 0)
        #for i in range(4):
        #frame2 = cv2.bilateralFilter (frame2, 9, 21, 31)
        frame2 = cv2.adaptiveThreshold(frame2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        _,contours,_ = cv2.findContours(frame2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        
        
#         frame2 = cv2.drawContours(frame2, contours, -1, (0,255,0), 3)
        
#        cv2.imshow("contours", frame2)
        if(len(contours) > 0):
            cont = sorted(contours, key=cv2.contourArea)
            cont = cont[-1]
            frame3 = frame2.copy()
            if(cv2.contourArea(cont) > 420):
                x,y,w,h = cv2.boundingRect(cont)
                cv2.rectangle(org, (x,y), (x+w, y+h), (255,0,255), 3)
                cv2.drawContours(frame3, [cont], -1, 255, -1)
                #print("drawing!")
                ((x2,y2),(w2,h2), angle) = cv2.minAreaRect(cont)
                #print(angle)
                if(w2 > 0 and h2 > 0):
                    #cv2.circle(frame, (int(y2 + h2//2), int(x2 + w2//2)), int(2), 100, -1)
                    frame3 = cv2.warpAffine(frame3, cv2.getRotationMatrix2D((x2, y2), angle, 1.0), (int(640),int(480)))
                    #x3,y3,w3,h3 = cv2.boundingRect(frame3)
                    #frame3 = frame3[y3:y3+h3, x3:x3+w3]
                #print(type(rect))
                #print(rect.shape)
                _,contours2,_ = cv2.findContours(frame3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cont2 = max(contours2, key=cv2.contourArea)
                #print(cv2.contourArea(cont2))
                #if(cv2.contourArea(cont2) > 500):
                x3,y3,w3,h3 = cv2.boundingRect(cont2)
                #cv2.drawContours(frame3, [cont2], -1, 255, -1)
                frame3 = frame3[y3 : y3+h3, x3 : x3+w3]
                resized = cv2.resize(frame3, (resize_size,resize_size), interpolation=cv2.INTER_NEAREST)
                resized[resized > pix_thresh] = 255
                resized[resized < pix_thresh] = 0
                cv2.imshow("resized", resized)
                #print(resized.shape)
                resized2 = resized.reshape((1, resize_size**2))
                resized2 = resized2.astype(np.float32)
                min_score = 10000000000
                const_min_score = 3500000
                best_img = resized
                letter = '',0
                for i in range(4):
                    _,result, nears, dists = knn.findNearest(resized2,5)
                    #print(nears)
                    #print(dists)
                    if(dists[0][0] < min_score and dists[0][0] < const_min_score):
                        letter = chr(int(result)),dists[0][0]
                        min_score = dists[0][0]
                        best_img = resized
                        
                    #resized = resized.astype(np.uint8)
                    resized = cv2.rotate(resized, cv2.ROTATE_90_CLOCKWISE)
                    resized2 = resized.reshape((1, resize_size**2)).astype(np.float32)
                    #resized2 = resized2.astype(np.float32)
                    
                cv2.putText(org, letter[0].upper(), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,0,0), 10)
                cv2.putText(org, str(letter[1]), (200,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 5)
                #print(letter)
                #print(nears)
                #print(dists)
                #print(chr(int(result)))
        for i in range(3):
            if len(bounding_rect[i]) > 0:
                x = bounding_rect[i][0]
                y = bounding_rect[i][1]
                w = bounding_rect[i][2]
                h = bounding_rect[i][3]
                if i % 3 == 0:
                    cv2.rectangle(org, (x,y), (x+w, y+h), (0,0,255), 3)
                elif i % 3 == 1:
                    cv2.rectangle(org, (x,y), (x+w, y+h), (0,255,0), 3)
                else:
                    cv2.rectangle(org, (x,y), (x+w, y+h), (0,255,255), 3)


                
        if camera_num:
            cv2.imshow("org", org)
        else:
            cv2.imshow("org1", org)
            
        camera_num = not(camera_num)

        ui = cv2.waitKey(1) & 0xFF
        if ui == ord('q'):
            break
        elif ui == ord(' '):
            output = label_img(best_img)
            if(len(output) > 1):
                label_txt = np.append(label_txt, output[0], axis = 0)
                feature_txt = np.append(feature_txt, output[1], axis = 0)
                knn.train(feature_txt, cv2.ml.ROW_SAMPLE, label_txt)
                #print(label_txt)
                #print(feature_txt)
    # print("write? (y/n): ")
    # if str(input())[0] == 'y':  
    #     feature_txt[feature_txt > pix_thresh] = 255
    #     feature_txt[feature_txt < pix_thresh] = 0
    #     np.savetxt("label.txt", label_txt)
    #     np.savetxt("feature.txt", feature_txt)
    #     print('written')

