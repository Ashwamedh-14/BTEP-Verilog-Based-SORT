from ultralytics import YOLO    

def main():
    model = YOLO("yolov8n.pt")
    results = model("DJI_0108.MP4", classes=[5])
    
    for r in results:
        for box in r.boxes:
            print(box.xywh, box.conf)
    print("Hello from video!")


if __name__ == "__main__":
    main()
