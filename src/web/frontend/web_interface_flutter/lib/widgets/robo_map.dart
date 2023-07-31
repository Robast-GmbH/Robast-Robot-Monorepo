import 'package:flutter/material.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:zoom_widget/zoom_widget.dart';

class RoboMap extends StatefulWidget {
  const RoboMap({super.key, required this.controller});
  final MapController controller;
  @override
  State<RoboMap> createState() => _RoboMapState();
}

class _RoboMapState extends State<RoboMap> {
  Offset tempPos = const Offset(0, 0);

  @override
  Widget build(BuildContext context) {
    return Zoom(
      maxScale: 5,
      initTotalZoomOut: true,
      child: SizedBox(
        width: 827,
        height: 355,
        child: GestureDetector(
          onTapDown: (details) {
            tempPos = details.localPosition;
          },
          onLongPress: () {
            widget.controller.position = tempPos;
            setState(() {});
          },
          child: Stack(
            children: [
              Image.asset("assets/RL_Tiplu_6.png"),
              if (widget.controller.position != null) ...[
                Positioned(
                  left: widget.controller.position!.dx - 8,
                  top: widget.controller.position!.dy - 8,
                  child: Container(
                    decoration: const BoxDecoration(
                      borderRadius: BorderRadius.all(Radius.circular(4)),
                      color: Colors.red,
                    ),
                    child: const Icon(
                      Icons.clear,
                      size: 16,
                    ),
                  ),
                ),
              ]
            ],
          ),
        ),
      ),
    );
  }
}
