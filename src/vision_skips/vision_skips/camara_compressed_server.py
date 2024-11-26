import numpy as np
import cv2 as cv
import socket

# Configuración del modelo YOLO
LABELS_FILE = "labels.names"
CONFIG_FILE = "yolo.cfg"
WEIGHTS_FILE = "yolo.weights"

# Cargamos los nombres de las clases
with open(LABELS_FILE, "r") as f:
    labels = f.read().strip().split("\n")

# Cargar YOLO
net = cv.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)
layer_names = net.getLayerNames()
unconnected_layers = net.getUnconnectedOutLayers()
if isinstance(unconnected_layers, np.ndarray) and unconnected_layers.ndim > 1:
    output_layers = [layer_names[i[0] - 1] for i in unconnected_layers]
else:
    output_layers = [layer_names[unconnected_layers[0] - 1] if isinstance(unconnected_layers, np.ndarray) else layer_names[unconnected_layers - 1]]

# Configuración de conexión
HOST = "0.0.0.0"
PORT = 10002

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Servidor escuchando en {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        print(f"Conectado por {addr}")
        b_imagen = bytearray()
        with conn:
            # Leer tamaño de imagen
            data = conn.recv(4)
            if not data:
                continue
            img_size = int.from_bytes(data, byteorder="little")

            # Leer datos de imagen
            while len(b_imagen) < img_size:
                data = conn.recv(4096)
                if not data:
                    break
                b_imagen += data

            img_enc = np.frombuffer(b_imagen, dtype='uint8')
            color = cv.imdecode(img_enc, cv.IMREAD_COLOR)

            # Detección de objetos usando YOLO
            blob = cv.dnn.blobFromImage(color, 1 / 255.0, (416, 416), swapRB=True, crop=False)
            net.setInput(blob)
            detections = net.forward(output_layers)

            h, w = color.shape[:2]
            boxes, confidences, class_ids = [], [], []
            detected_labels = []  # Lista para almacenar nombres de objetos detectados

            for output in detections:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.8:  # Filtrar detecciones con confianza >= 0.8
                        box = detection[0:4] * np.array([w, h, w, h])
                        (center_x, center_y, width, height) = box.astype("int")
                        x, y = int(center_x - width / 2), int(center_y - height / 2)
                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
                        detected_labels.append(labels[class_id])  # Agregar el nombre del objeto detectado

            indices = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

            # Dibujar las detecciones en la imagen
            for i in indices:
                idx = i[0] if isinstance(i, (list, tuple)) else i
                (x, y) = (boxes[idx][0], boxes[idx][1])
                (w, h) = (boxes[idx][2], boxes[idx][3])
                color_box = [int(c) for c in np.random.randint(0, 255, size=(3,))]
                cv.rectangle(color, (x, y), (x + w, y + h), color_box, 2)
                text = f"{labels[class_ids[idx]]}: {confidences[idx]:.2f}, W:{w}px H:{h}px"
                cv.putText(color, text, (x, y - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, color_box, 2)

            # Enviar datos como JSON-like formato binario
            data_dict = {
                "labels": detected_labels,
                "image": cv.imencode('.jpg', color)[1].tobytes()
            }
            
            # Convertir datos en binario y enviarlos
            import pickle
            serialized_data = pickle.dumps(data_dict)
            conn.sendall(len(serialized_data).to_bytes(4, byteorder="little"))
            conn.sendall(serialized_data)
