from typing import Iterable, List, Callable, Tuple, TypeVar, cast, Any, \
    Union, Iterator, Generic

from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.types import TubRecord, TubRecordDict

X = TypeVar('X', covariant=True)
Y = TypeVar('Y', covariant=True)
GX = TypeVar('GX', covariant=True)
XOut = TypeVar('XOut', covariant=True)
YOut = TypeVar('YOut', covariant=True)



class TubDataset(object):
    def __init__(self, paths: List[str], config: Any) -> None:
        self.paths = paths
        self.config = config
        self.tubs = [Tub(path) for path in self.paths]
        self.records: List[TubRecord] = list()

    def load_records(self) -> List[TubRecord]:
        self.records.clear()
        for tub in self.tubs:
            for record in tub:
                underlying = cast(TubRecordDict, record)
                tub_record = TubRecord(self.config, tub.base_path, underlying)
                self.records.append(tub_record)

        return self.records

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, item: Union[int, slice]) -> TubRecord:
        return self.records[item]

    def __iter__(self) -> Iterator[TubRecord]:
        for record in self.records:
            yield record



class TubSequence(object):
    def __init__(self, records: List[TubRecord]) -> None:
        self.records = records

    def __iter__(self) -> Iterable[TubRecord]:
        return iter(self.records)

    def __len__(self):
        return len(self.records)

    def __getitem__(self, item):
        return self.records[item]

    def build_pipeline(self,
                       x_transform: Callable[[TubRecord], X],
                       y_transform: Callable[[TubRecord], Y]) -> Iterable[Tuple[X, Y]]:

        iterator = self.__iter__()
        for record in iterator:
            x = x_transform(record)
            y = y_transform(record)
            yield x, y

    @classmethod
    def map_pipeline(
            cls,
            x_transform: Callable[[X], XOut],
            y_transform: Callable[[Y], YOut],
            pipeline: Iterable[Tuple[X, Y]]) -> Iterable[Tuple[XOut, YOut]]:

        for record in iter(pipeline):
            x, y = record
            yield x_transform(x), y_transform(y)

    @classmethod
    def map_pipeline_factory(
            cls,
            x_transform: Callable[[X], XOut],
            y_transform: Callable[[Y], YOut],
            factory: Callable[[], Iterable[Tuple[X, Y]]]) -> Iterable[Tuple[XOut, YOut]]:

        pipeline = factory()
        return cls.map_pipeline(pipeline=pipeline, x_transform=x_transform, y_transform=y_transform)


class BasePipeline(Generic[GX], Iterable):

    def __init__(self) -> None:
        pass

    def __len__(self) -> int:
        pass

    def __iter__(self) -> Iterator[GX]:
        pass


class Pipeline(BasePipeline[Tuple[X, Y]]):
    def __init__(self,
                 sequence: Union[TubSequence, BasePipeline],
                 x_transform: Callable[[TubRecord], X],
                 y_transform: Callable[[TubRecord], Y]) -> None:
        super().__init__()
        self.sequence = sequence
        self.x_transform = x_transform
        self.y_transform = y_transform

    def __iter__(self) -> Iterator[Tuple[X, Y]]:
        for record in self.sequence:
            yield self.x_transform(record), self.y_transform(record)

    def __len__(self) -> int:
        return len(self.sequence)

    def __getitem__(self, item):
        record = self.sequence[item]
        return self.x_transform(record), self.y_transform(record)