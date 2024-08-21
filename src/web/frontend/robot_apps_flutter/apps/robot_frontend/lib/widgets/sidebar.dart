import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:robot_frontend/models/sidebar_menu_point.dart';

class Sidebar extends StatefulWidget {
  const Sidebar({required this.sidebarMenuPoints, required this.onMenuPointSelected, this.user, super.key});

  final List<SidebarMenuPoint> sidebarMenuPoints;
  final void Function(int) onMenuPointSelected;
  final User? user;

  @override
  State<Sidebar> createState() => _SidebarState();
}

class _SidebarState extends State<Sidebar> {
  int selectedMainMenuIndex = 0;

  bool isSidebarExpanded = false;
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16),
      child: Container(
        width: isSidebarExpanded ? 300 : null,
        height: double.infinity,
        decoration: BoxDecoration(color: Colors.black.withOpacity(0.5), borderRadius: BorderRadius.circular(16)),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            buildMenuButton(
                icon: RotatedBox(
                  quarterTurns: isSidebarExpanded ? 3 : 1,
                  child: Icon(
                    Icons.expand_less,
                    color: Colors.white,
                    size: 40,
                  ),
                ),
                title: 'Einklappen',
                onPressed: () {
                  setState(() {
                    isSidebarExpanded = !isSidebarExpanded;
                  });
                },
                isSelected: false,
                isExpanded: isSidebarExpanded),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                SizedBox(height: 8),
                ...List.generate(
                    widget.sidebarMenuPoints.length,
                    (index) => buildMenuButton(
                          icon: Icon(
                            widget.sidebarMenuPoints[index].icon,
                            color: index == selectedMainMenuIndex ? Colors.white : Colors.white70,
                            size: 40,
                          ),
                          title: widget.sidebarMenuPoints[index].title,
                          onPressed: () {
                            widget.onMenuPointSelected(index);
                            setState(() {
                              selectedMainMenuIndex = index;
                            });
                          },
                          isSelected: index == selectedMainMenuIndex,
                          isExpanded: isSidebarExpanded,
                        )),
              ],
            ),
            Padding(
              padding: const EdgeInsets.all(16),
              child: Container(
                width: isSidebarExpanded ? double.infinity : null,
                decoration: BoxDecoration(color: Color.fromARGB(255, 85, 186, 211), borderRadius: BorderRadius.circular(8)),
                child: Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    children: [
                      Icon(
                        Icons.account_circle,
                        size: 48,
                      ),
                      if (isSidebarExpanded) ...[
                        SizedBox(
                          width: 8,
                        ),
                        Text('${widget.user?.firstName[0]}. ${widget.user?.lastName}', style: TextStyle(fontSize: 24, color: Colors.white)),
                      ]
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget buildMenuButton({required Widget icon, required String title, required VoidCallback onPressed, required bool isSelected, required bool isExpanded}) {
    return Column(
      children: [
        InkWell(
          onTap: onPressed,
          child: Container(
            decoration: BoxDecoration(
              gradient: isSelected
                  ? LinearGradient(
                      begin: Alignment.centerLeft,
                      end: Alignment.centerRight,
                      colors: [
                        Colors.white.withOpacity(0.3),
                        Colors.white.withOpacity(0.05),
                      ],
                    )
                  : null,
              border: Border(left: BorderSide(color: isSelected ? Colors.white.withOpacity(0.8) : Colors.transparent, width: 4)),
            ),
            child: Padding(
              padding: const EdgeInsets.only(right: 4) + const EdgeInsets.all(24),
              child: Row(children: [
                icon,
                if (isExpanded) ...[
                  SizedBox(width: 16),
                  Text(
                    title,
                    style: TextStyle(
                      fontSize: 24,
                      color: Colors.white,
                    ),
                  )
                ],
              ]),
            ),
          ),
        ),
        SizedBox(height: 8),
      ],
    );
  }
}
