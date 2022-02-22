FROM osrf/ros:foxy-desktop

COPY ./run.bash /
RUN rm /opt/ros/foxy/lib/libfastrtps*
COPY ./bin/fastdds/* /opt/ros/foxy/lib/
RUN chmod +x /run.bash


ENTRYPOINT ["/run.bash"]
CMD ["bash"]
