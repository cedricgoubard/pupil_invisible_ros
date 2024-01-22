FROM personalroboticsimperial/prl:noetic

SHELL [ "bash", "-c" ]

RUN python -m pip install asyncio pupil-labs-realtime-api

CMD ["bash"]